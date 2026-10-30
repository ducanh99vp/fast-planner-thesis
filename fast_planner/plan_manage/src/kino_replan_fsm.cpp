/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/




#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("fsm/takeoff_height", takeoff_height_, -1.0);  // [Luan van - M3]
  nh.param("fsm/goal_z", goal_z_, 1.0);   // [CD2] goc HKUST ep cung z dich = 1.0
  nh.param("fsm/global_guide", global_guide_, false);   // [CD1] false = ban goc
  nh.param("fsm/goal_retry", goal_retry_, false);       // [CD2] false = ban goc
  nh.param("fsm/guide_dyn_soft", guide_dyn_soft_, false);   // [CD2 P3L]
  nh.param("fsm/dyn_dodge", dyn_dodge_, -1.0);          // [CD2] <= 0 = tat (ban goc)
  nh.param("fsm/stop_on_fail", stop_on_fail_, false);   // [CD2 25/10] false = ban goc (bay tiep quy dao cu)
  nh.param("fsm/takeoff_watch", takeoff_watch_, false);  // [CD2 P3]
  nh.param("fsm/takeoff_min_z", takeoff_min_z_, 0.4);
  nh.param("fsm/takeoff_margin", takeoff_margin_, 0.5);
  nh.param("fsm/goal_dyn_T", goal_dyn_T_, 0.0);          // [CD2 P3 27/10] 0 = tat
  nh.param("fsm/goal_dyn_occ", goal_dyn_occ_, 0.5);
  nh.param("fsm/goal_dyn_free", goal_dyn_free_, 0.7);
  nh.param("fsm/goal_dyn_range", goal_dyn_range_, 5.0);
  nh.param("fsm/guide_dist", guide_dist_, 6.0);         // [CD1] > search/horizon: A* dung o chan troi, diem cuoi mem nhu ban goc
  nh.param("fsm/guide_vel", guide_vel_, 1.0);           // [CD1]
  nh.param("fsm/guide_hidden_r", guide_hidden_r_, 2.5); // [CD1]
  nh.param("fsm/guide_replan_t", guide_replan_t_, 1.0); // [CD1]
  nh.param("fsm/replan_latency", replan_latency_, -1.0);   // [CD2 Indoor] <= 0 = ban goc
  takeoff_planned_ = false;

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, goal_z_;  // [CD2]

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  /* [CD2 30/10] Dich NGOAI ban do (chay tay ngoai troi: dich (-9.37, -10.82) trong khi ban do chi toi
     y = -10): duong toan cuc that bai, UAV bay ra ngoai mep roi KET VINH VIEN (diem xuat phat ngoai ban
     do -> A* hong ngay buoc dau, lap lai ~70 lan/s). Keo dich vao trong ban do, cach mep >= 1 m. */
  {
    Eigen::Vector3d ori, size;
    planner_manager_->edt_environment_->sdf_map_->getRegion(ori, size);
    const Eigen::Vector3d dich_goc = end_pt_;
    for (int i = 0; i < 2; ++i)
      end_pt_(i) = min(max(end_pt_(i), ori(i) + 1.0), ori(i) + size(i) - 1.0);
    if ((end_pt_ - dich_goc).norm() > 1e-6)
      ROS_WARN("[CD2] dich (%.2f, %.2f) ngoai hoac sat mep ban do -> keo vao (%.2f, %.2f)",
               dich_goc(0), dich_goc(1), end_pt_(0), end_pt_(1));
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;
  goal_true_   = end_pt_;   // [CD2]

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(needTakeoff() ? TAKEOFF : GEN_NEW_TRAJ, "TRIG");  // [Luan van - M3]
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[7] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW", "TAKEOFF" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[7] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW", "TAKEOFF" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(needTakeoff() ? TAKEOFF : GEN_NEW_TRAJ, "FSM");  // [Luan van - M3]
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      if (replan_latency_ > 0.0 && info->traj_id_ > 0) {
        /* [CD2 16/10] Lap lai sau that bai: lap tu trang thai LENH cua quy dao dang bay tai luc
           ban giao (giong REPLAN_TRAJ), khong tu odom voi gia toc 0 va moc = luc bat dau lap — hai
           thu do lam lenh nhay. Bao traj_server keo dai quy dao cu toi luc ban giao. */
        double t = (ros::Time::now() - info->start_time_).toSec() + replan_latency_;
        t        = min(max(t, 0.0), info->duration_);
        start_pt_     = info->position_traj_.evaluateDeBoorT(t);
        start_vel_    = info->velocity_traj_.evaluateDeBoorT(t);
        start_acc_    = info->acceleration_traj_.evaluateDeBoorT(t);
        start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t)[0];
        start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t)[0];
        start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t)[0];
        planner_manager_->desired_start_time_ = ros::Time::now() + ros::Duration(replan_latency_);
        std_msgs::Empty replan_msg;
        replan_pub_.publish(replan_msg);
      } else {
        start_pt_  = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();

        Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
        start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
        start_yaw_(1) = start_yaw_(2) = 0.0;
      }

      bool success = callKinodynamicReplan();
      const ros::Time t_ban_giao = planner_manager_->desired_start_time_;   // [CD2 25/10]
      planner_manager_->desired_start_time_ = ros::Time(0);   // [CD2 16/10]
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else if (stopIfUnsafe(t_ban_giao)) {
        changeFSMExecState(EXEC_TRAJ, "STOP");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
        /* [CD1] Quy dao het o DICH TAM, chua toi dich that: lap tiep, khong dung lai. */
        /* [CD2] Loi goc (M5): het quy dao bi coi la toi dich. Vat can dong lam quy dao ket thuc
           truoc dich (do 16/10: I3 hat 9, I4-R2 hat 5 treo cach dich 1.0-1.3 m toi het gio). */
        /* [CD2] goal_retry_: xet theo dich THAT. Neu dang dung o dich TAM (dich that con bi vet
           quet de) thi cho tai cho: phep kiem 20 Hz tra lai dich that khi no trong, hoac doi
           dich tam khi vet quet lan toi cho UAV dung. */
        const Eigen::Vector3d& g = goal_retry_ ? goal_true_ : end_pt_;
        if ((global_guide_ || goal_retry_) && (g - odom_pos_).norm() > 0.5) {
          if (goal_retry_ && (end_pt_ - goal_true_).norm() > 1e-3 && (end_pt_ - odom_pos_).norm() < 0.5)
            return;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_ &&
                 !(global_guide_ && info->duration_ - t_cur < guide_replan_t_)) {
        /* [CD1] Dich tam gan (truoc cua, goc) -> quy dao ngan hon replan_thresh_: neu cho du
           1.5 m moi lap lai thi UAV bay het quy dao, phanh ve ~0 roi moi lap tu dau (do 15/10:
           I4-R2 khung o cua C, 1.6 -> 0.11 m/s). Con < guide_replan_t_ giay thi lap lai ngay. */
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      /* [CD2 Indoor] Lap ke hoach tu trang thai UAV SE CO sau replan_latency_ giay (UAV van
         bay quy dao cu trong luc cho), va gan moc thoi gian dung thoi diem do. Quy dao moi
         vi vay noi tiep quy dao cu, du lap ke hoach mat bao lau trong pham vi do. */
      if (replan_latency_ > 0.0) {
        /* Ban giao luc (bay gio + replan_latency_), KHONG phai (dau quy dao + latency): neu
           quy dao hien tai con dang CHO (lap lai lien tiep, t_cur < 0) thi cong don lam moc
           ban giao troi xa dan va bi ho voi quy dao cu da cat -> lenh v = 0 -> giat (do 13/10). */
        t_cur = min(max(t_cur + replan_latency_, 0.0), info->duration_);
        /* [CD2 24/10] Quy dao cu DA HET (UAV dang treo o diem cuoi, vd cho o dich tam) thi moc
           "diem cuoi quy dao cu" da nam trong QUA KHU: traj_server bat dau quy dao moi giua chung,
           lenh nhay -> I4-R3 hat 6 nghieng 54 do, 2.06 m/s (moc lui 1.0 s). Khong bao gio dat moc
           som hon bay gio + replan_latency_. */
        ros::Time t_start = info->start_time_ + ros::Duration(t_cur);
        const ros::Time t_min = time_now + ros::Duration(replan_latency_);
        if (t_start < t_min) t_start = t_min;
        planner_manager_->desired_start_time_ = t_start;
      }

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      const ros::Time t_ban_giao = planner_manager_->desired_start_time_;   // [CD2 25/10]
      planner_manager_->desired_start_time_ = ros::Time(0);   // [CD2 Indoor] khong de sot sang lan sau
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else if (stopIfUnsafe(t_ban_giao)) {
        changeFSMExecState(EXEC_TRAJ, "STOP");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    // [Luan van - M3] Cat canh thang dung len takeoff_height_ roi moi lap ke hoach toi dich.
    case TAKEOFF: {
      if (!takeoff_planned_) {
        if (!planTakeoffTraj()) {  // da o du cao: bo qua pha cat canh
          changeFSMExecState(GEN_NEW_TRAJ, "TAKEOFF");
          break;
        }
        takeoff_planned_ = true;
        cout << "[FSM]: takeoff to z = " << takeoff_height_ << endl;
        break;
      }
      LocalTrajData* info    = &planner_manager_->local_data_;
      double         t_cur   = (ros::Time::now() - info->start_time_).toSec();
      bool           reached = fabs(odom_pos_(2) - takeoff_height_) < 0.1 && odom_vel_.norm() < 0.2;
      /* [CD2 P3] I3T2B hat 3 (26/10): vat can lan giua di qua dung diem xuat phat luc UAV vua cat canh
         xong (t 2.6 s), planner chua duoc phep lam gi -> cham -0.21. */
      if (!reached && takeoff_watch_ && odom_pos_(2) >= takeoff_min_z_) {
        const double t_left = max(0.0, info->duration_ - t_cur) + 1.0;
        const double tnow   = ros::Time::now().toSec();
        bool nguy = false;
        for (double dt = 0.0; dt <= t_left + 1e-6 && !nguy; dt += 0.1)
          for (double z = odom_pos_(2); z <= takeoff_height_ + 1e-6 && !nguy; z += 0.2)
            if (planner_manager_->edt_environment_->sdf_map_->dynPredMinDist(
                    Eigen::Vector3d(odom_pos_(0), odom_pos_(1), z), tnow + dt) < takeoff_margin_)
              nguy = true;
        if (nguy) {
          cout << "[CD2 P3] vat can dong sap di qua cot cat canh -> ket thuc cat canh som o z = " << odom_pos_(2) << endl;
          takeoff_planned_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "TAKEOFF");
          break;
        }
      }
      if (reached) {
        takeoff_planned_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "TAKEOFF");
      } else if (t_cur > info->duration_ + 3.0) {
        takeoff_planned_ = false;  // quy dao da het ma chua toi do cao: dung lai quy dao cat canh
      }
      break;
    }
  }
}

/* [CD2 P3 27/10] Khoang cach nho nhat tu p toi moi vat can du doan trong goal_dyn_T_ giay toi. */
double KinoReplanFSM::goalDynClear(const Eigen::Vector3d& p) {
  const double tnow = ros::Time::now().toSec();
  double dmin = 1e9;
  for (double dt = 0.0; dt <= goal_dyn_T_ + 1e-6; dt += 0.1)
    dmin = min(dmin, planner_manager_->edt_environment_->sdf_map_->dynPredMinDist(p, tnow + dt));
  return dmin;
}

/* [CD2 P3 27/10] Dich tam GAN dich that nhat (vong 0.5 m, 12 huong, z +-0.3 nhu ban goc) vua trong tinh
   vua khong bi vat can dong quet qua; cung ban kinh thi chon diem thoang nhat. */
bool KinoReplanFSM::findDynSafeGoal(const Eigen::Vector3d& gc, Eigen::Vector3d& goal) {
  auto edt_env = planner_manager_->edt_environment_;
  for (double r = 0.5; r <= 2.5 + 1e-3; r += 0.5) {
    double best = -1.0;
    for (double theta = -90; theta <= 270; theta += 30)
      for (double nz = 0.3; nz >= -0.3 - 1e-6; nz -= 0.3) {
        Eigen::Vector3d p(gc(0) + r * cos(theta / 57.3), gc(1) + r * sin(theta / 57.3), gc(2) + nz);   // evaluateCoarseEDT can tham chieu khong const
        const double ds = edt_env->evaluateCoarseEDT(p, -1.0);
        if (ds <= 0.5) continue;
        const double dd = goalDynClear(p);
        if (dd < goal_dyn_free_) continue;
        if (min(ds, dd) > best) {
          best = min(ds, dd);
          goal = p;
        }
      }
    if (best > 0.0) return true;
  }
  return false;
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  if (exec_state_ == TAKEOFF) return;  // [Luan van - M3] khong replan/doi dich trong luc cat canh
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    /* [CD2] Goc HKUST: dich bi chiem thi DOI HAN dich sang diem trong nhat quanh do — hop voi
       vat can tinh. Vet quet vat can dong chi de len dich mot luc, nhung dich van bi doi mai:
       UAV toi dich gia cach 1.0-1.3 m roi WAIT_TARGET, dung yen khong ne toi het gio (do 16/10:
       I3 hat 9, I4-R2 hat 5, 4/4 luot co "change goal"). Tra lai dich that khi no trong
       (nguong 0.5 > 0.3 de khong doi qua doi lai). */
    if (goal_retry_ && (end_pt_ - goal_true_).norm() > 1e-3 &&
        edt_env->evaluateCoarseEDT(goal_true_, -1.0) > 0.5 &&
        (goal_dyn_T_ <= 0.0 || goalDynClear(goal_true_) >= goal_dyn_free_)) {   // [CD2 P3 27/10]
      cout << "[CD2] dich that da trong, tra lai dich." << endl;
      end_pt_ = goal_true_;
      end_vel_.setZero();
      visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      if (exec_state_ == EXEC_TRAJ) changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }

    /* [CD2 P3 27/10] vat can dong sap quet qua dich (dang dung) -> dich tam gan nhat an toan */
    if (goal_dyn_T_ > 0.0 && goal_retry_ && (odom_pos_ - goal_true_).norm() < goal_dyn_range_ &&
        goalDynClear(end_pt_) < goal_dyn_occ_) {
      Eigen::Vector3d g;
      if (findDynSafeGoal(goal_true_, g)) {
        cout << "[CD2 P3] vat can dong sap quet qua dich -> dich tam (" << g(0) << ", " << g(1) << ")" << endl;
        end_pt_ = g;
        end_vel_.setZero();
        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
        if (exec_state_ == EXEC_TRAJ) changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      }
    }

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;
      const Eigen::Vector3d gc = goal_retry_ ? goal_true_ : end_pt_;   // [CD2] tim quanh dich THAT

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = gc(0) + r * cos(theta / 57.3);
            new_y = gc(1) + r * sin(theta / 57.3);
            new_z = gc(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    /* [CD2 16/10] Quy dao vua lap con CHO toi luc ban giao (replan_latency): de no bat dau da.
       Neu kiem va lap lai o day, 20 Hz, quy dao moi bi thay truoc khi kip dung, UAV bay het quy
       dao cu da bi cat roi phanh, toi khi ngung lai thi lenh nhay (do 16/10: I3 hat 2, 98 lan lap
       lai lien tiep, nghieng 72-101 do). */
    if (replan_latency_ > 0.0 && ros::Time::now() < info->start_time_) return;
    double dist;
    /* [CD2] Quy dao ne ngang CO Y cat qua vet quet cua vat can dang ne: chi bat vat can that,
       neu khong no bi thay ngay (0.2-0.6 s) bang quy dao huong ve dich -> doi chieu, nghieng
       25-33 do (do 16/10, thu nhanh P2HC3). */
    bool   safe = planner_manager_->checkTrajCollision(dist, dodge_traj_);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

/* [CD1] A* toan cuc tu diem xuat phat toi dich that tren ban do co tri nho; tra ve diem dau
   tien tren duong cach diem xuat phat (do theo duong) >= guide_dist_. Duong ngan hon thi tra
   ve dich that; khong tim duoc duong thi quay ve hanh vi goc (nham thang dich that).
   Van toc tai dich tam = guide_vel_ theo huong duong: quy dao cuc bo khong phai phanh ve 0
   o moi dich tam (lap lai moi 1.5 m nen UAV khong bao gio toi dich tam). */
Eigen::Vector3d KinoReplanFSM::guideTarget(Eigen::Vector3d& target_vel) {
  /* Giu duong cu neu: van dan toi dich hien tai, UAV con cach duong < 1 m, va phan con lai
     van trong tren ban do da nho. Tranh doi qua doi lai giua hai duong dai ngang nhau (I5 doi
     xung) va tranh chay A* toan cuc o moi lan lap ke hoach. */
  size_t k0    = 0;
  bool   reuse = false;
  if (guide_path_.size() >= 2 && (guide_path_.back() - end_pt_).norm() < 0.5) {
    double dmin = 1e9;
    for (size_t i = 0; i < guide_path_.size(); ++i) {
      const double d = (guide_path_[i] - start_pt_).norm();
      if (d < dmin) { dmin = d; k0 = i; }
    }
    /* [CD2 P3L] co chi phi mem vat can dong thi lap lai duong toan cuc moi lan (lan dong thay doi theo
       thoi gian); khong thi giu duong cu nhu CD1. */
    reuse = !guide_dyn_soft_ && dmin < 1.0 && planner_manager_->globalPathFree(guide_path_, k0);
  }
  if (!reuse) {
    vector<Eigen::Vector3d> path;
    if (!planner_manager_->planGlobalPath(start_pt_, end_pt_, path)) {
      ROS_WARN("[global guide] khong co duong toan cuc, nham thang dich");
      guide_path_.clear();
      return end_pt_;
    }
    guide_path_ = path;
    k0          = 0;
  }
  visualization_->drawGeometricPath(guide_path_, 0.08, Eigen::Vector4d(0.0, 0.8, 0.2, 1.0), 1);
  /* Dich tam = diem XA NHAT tren duong (<= guide_dist_ tinh theo duong) ma doan thang tu
     diem xuat phat toi do con trong. Khong lay diem khuat sau goc tuong: A* cuc bo nham thang
     toi dich tam nen se cat goc ep vao tuong (do 14/10: I5 va cham o mat trong vach tui). */
  const size_t n    = guide_path_.size();
  size_t       best = k0;
  double       s    = (guide_path_[k0] - start_pt_).norm();
  for (size_t i = k0 + 1; i < n; ++i) {
    s += (guide_path_[i] - guide_path_[i - 1]).norm();
    if (s > guide_dist_) break;
    /* Diem khuat nhung o gan (<= guide_hidden_r_) van chon: A* cuc bo tim duoc duong that toi
       no (vd. xuyen qua cua). Chi diem XA moi bat buoc nhin thay (tranh cat goc o I5). */
    if ((guide_path_[i] - start_pt_).norm() > guide_hidden_r_ &&
        !planner_manager_->segmentFree(start_pt_, guide_path_[i]))
      break;
    best = i;
  }
  if (best + 1 >= n) return end_pt_;   // nhin thay dich that: ve dich, van toc cuoi = end_vel_
  if (best == k0) best = k0 + 1;
  /* [CD2 21/10] Tam nhin bo qua vet quet nen dich tam co the roi VAO vet quet: doi TIEN doc duong
     (toi 2 m) toi o trong — A* cuc bo phai vong qua vat can de toi do, khong dung truoc mat no;
     khong co thi doi LUI. */
  if (planner_manager_->guide_ignore_dyn_ && planner_manager_->inDynSweep(guide_path_[best])) {
    size_t q = best;
    double ds = 0.0;
    while (q + 1 < n && ds < 2.0 && planner_manager_->inDynSweep(guide_path_[q])) {
      ds += (guide_path_[q + 1] - guide_path_[q]).norm();
      ++q;
    }
    if (planner_manager_->inDynSweep(guide_path_[q])) {
      q = best;
      while (q > k0 + 1 && planner_manager_->inDynSweep(guide_path_[q])) --q;
    }
    best = q;
    if (best + 1 >= n) return end_pt_;
  }
  /* Van toc tai dich tam theo huong duong, giam theo khoang cach: dich tam gan (sau goc) thi
     quy dao ngan, ep 1 m/s o cuoi se giat. */
  const double dsub = (guide_path_[best] - start_pt_).norm();
  target_vel = guide_vel_ * std::min(1.0, dsub / guide_dist_) *
               (guide_path_[std::min(best + 1, n - 1)] - guide_path_[best - 1]).normalized();
  return guide_path_[best];
}

bool KinoReplanFSM::callKinodynamicReplan() {
  /* [CD1] Dan duong toan cuc: dich cua lan lap ke hoach cuc bo la diem tren duong toan cuc,
     khong phai dich that. end_pt_ van la dich that (dung de xet toi dich). */
  Eigen::Vector3d target_vel = end_vel_;
  Eigen::Vector3d target     = global_guide_ ? guideTarget(target_vel) : end_pt_;
  /* [CD2] Ne ngang: diem xuat phat nam trong vet quet PHIA TRUOC mot vat can dang lao toi ->
     lap ke hoach toi diem ne (vuong goc huong vat can, ngoai vet quet) thay vi toi dich; het
     quy dao ne thi EXEC_TRAJ lap tiep toi dich. Bong bong thoat noi rong cho het doan ne (doan
     do cat ngang vet quet cua chinh vat can dang ne). Van toc tai diem ne: 0.5 m/s huong ve
     dich nhung bo thanh phan quay lai lan vat can — dung han o diem ne roi moi bay tiep lam UAV
     doi chieu, nghieng 26-33 do (thu nhanh 16/10). */
  bool dodge = false;
  if (dyn_dodge_ > 0.0) {
    Eigen::Vector3d q;
    if (planner_manager_->dodgeTarget(start_pt_, end_pt_, dyn_dodge_, q) && (q - start_pt_).norm() > 0.25) {
      dodge  = true;
      target = q;
      target_vel.setZero();
      Eigen::Vector3d g = end_pt_ - q, out = q - start_pt_;
      g(2) = 0.0; out(2) = 0.0; out.normalize();
      if (g.dot(out) < 0.0) g -= g.dot(out) * out;   // khong quay lai lan
      if (g.norm() > 0.1) target_vel = 0.5 * g.normalized();
      planner_manager_->setStartEscape((q - start_pt_).norm() + 0.15);
      cout << "[CD2] ne ngang toi " << q.transpose() << endl;
    }
  }
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, target, target_vel);
  if (dodge) planner_manager_->setStartEscape(-1.0);
  if (plan_success) dodge_traj_ = dodge;

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    publishLocalTraj();

    /* visulization */
    auto info = &planner_manager_->local_data_;
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// [Luan van - M3] Tach tu callKinodynamicReplan() de pha cat canh dung chung.
void KinoReplanFSM::publishLocalTraj() {
  auto info = &planner_manager_->local_data_;

  /* publish traj */
  plan_manage::Bspline bspline;
  bspline.order      = 3;
  bspline.start_time = info->start_time_;
  bspline.traj_id    = info->traj_id_;

  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

  for (int i = 0; i < pos_pts.rows(); ++i) {
    geometry_msgs::Point pt;
    pt.x = pos_pts(i, 0);
    pt.y = pos_pts(i, 1);
    pt.z = pos_pts(i, 2);
    bspline.pos_pts.push_back(pt);
  }

  Eigen::VectorXd knots = info->position_traj_.getKnot();
  for (int i = 0; i < knots.rows(); ++i) {
    bspline.knots.push_back(knots(i));
  }

  Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  for (int i = 0; i < yaw_pts.rows(); ++i) {
    double yaw = yaw_pts(i, 0);
    bspline.yaw_pts.push_back(yaw);
  }
  bspline.yaw_dt = info->yaw_traj_.getInterval();

  bspline_pub_.publish(bspline);
}

// [Luan van - M3] Quy dao cat canh thang dung, dung truc tiep bang B-spline, KHONG qua A*/toi uu.
// Ly do: san ban do nam o z = 0; UAV dang dau tren san nam trong o vat can da phinh nen
// kinodynamic A* khong mo rong duoc nut nao (NO_PATH).
/* [CD2 25/10] P5 TOI THIEU. Ban goc (va ban chong bao lap lai 16/10): lap ke hoach that bai thi UAV
   BAY TIEP quy dao cu. Khi du doan (vet quet, phan xa o tuong) cho biet quy dao cu sap di vao vung bi
   chiem, bay tiep la lao vao do (I4-R3 hat 1: 2.6 s khong lap duoc ke hoach nao, quy dao cu dua UAV
   vao cua D dung luc hop bat lai -> cham). Chi khi do: thay bang quy dao PHANH DEU theo huong dang bay
   (gia toc 0.8 max_acc) roi treo tai cho; quy dao cu con an toan thi giu nguyen hanh vi cu. */
bool KinoReplanFSM::stopIfUnsafe(const ros::Time& t0) {
  if (!stop_on_fail_ || replan_latency_ <= 0.0) return false;
  double dist;
  if (planner_manager_->checkTrajCollision(dist)) return false;   // quy dao cu con an toan
  cout << "[CD2] lap ke hoach that bai + quy dao cu sap cham (" << dist << " m) -> phanh dung" << endl;
  return planStopTraj(t0);
}

bool KinoReplanFSM::planStopTraj(const ros::Time& t0) {
  auto&                 pp = planner_manager_->pp_;
  const double          a  = 0.8 * pp.max_acc_;
  const Eigen::Vector3d v0 = start_vel_;
  const double          s0 = v0.norm();
  const double          Tb = s0 / a;                       // thoi gian phanh
  const double          Th = 0.4;                          // treo them cho dung han
  const Eigen::Vector3d u  = s0 > 1e-3 ? Eigen::Vector3d(v0 / s0) : Eigen::Vector3d::Zero();
  const int             n  = max(4, int(ceil((Tb + Th) / 0.05)));
  const double          dt = (Tb + Th) / n;
  vector<Eigen::Vector3d> pts;
  for (int i = 0; i <= n; ++i) {
    const double t = min(i * dt, Tb);
    pts.push_back(start_pt_ + u * (s0 * t - 0.5 * a * t * t));
  }
  vector<Eigen::Vector3d> derivs = { v0, Eigen::Vector3d::Zero(), start_acc_, Eigen::Vector3d::Zero() };
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, pts, derivs, ctrl_pts);

  LocalTrajData* info      = &planner_manager_->local_data_;
  info->start_time_        = t0.isZero() ? ros::Time::now() : t0;
  info->position_traj_     = NonUniformBspline(ctrl_pts, 3, dt);
  info->velocity_traj_     = info->position_traj_.getDerivative();
  info->acceleration_traj_ = info->velocity_traj_.getDerivative();
  info->start_pos_         = info->position_traj_.evaluateDeBoorT(0.0);
  info->duration_          = info->position_traj_.getTimeSum();
  info->traj_id_ += 1;
  int             seg_num = max(1, int(ceil(info->duration_ / 0.3)));
  Eigen::MatrixXd yaw     = Eigen::MatrixXd::Constant(seg_num + 3, 1, start_yaw_(0));   // giu huong mui
  info->yaw_traj_.setUniformBspline(yaw, 3, info->duration_ / seg_num);
  info->yawdot_traj_    = info->yaw_traj_.getDerivative();
  info->yawdotdot_traj_ = info->yawdot_traj_.getDerivative();
  dodge_traj_ = false;
  publishLocalTraj();
  visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0.5, 0.0, 1), true, 0.2,
                              Eigen::Vector4d(1, 0.5, 0, 1));
  return true;
}

bool KinoReplanFSM::planTakeoffTraj() {
  const double dz = takeoff_height_ - odom_pos_(2);
  if (dz <= 0.0) return false;

  // Profile bac 5 s(tau) = 10tau^3 - 15tau^4 + 6tau^5: van toc, gia toc dau/cuoi = 0.
  // v_dinh = 1.875 dz/T, a_dinh = 5.7735 dz/T^2; gioi han bang mot nua max_vel/max_acc.
  auto&        pp = planner_manager_->pp_;
  const double T  = max(1.875 * dz / (0.5 * pp.max_vel_), sqrt(5.7735 * dz / (0.5 * pp.max_acc_)));
  const int    n  = max(3, int(ceil(T / 0.1)));
  const double dt = T / n;

  vector<Eigen::Vector3d> pts;
  vector<Eigen::Vector3d> derivs(4, Eigen::Vector3d::Zero());  // v0, v1, a0, a1
  for (int i = 0; i <= n; ++i) {
    double s = double(i) / n;
    pts.push_back(odom_pos_ + Eigen::Vector3d(0, 0, dz * s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)));
  }
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, pts, derivs, ctrl_pts);

  // Cap nhat local_data_ nhu FastPlannerManager::updateTrajInfo() (ham do la private)
  LocalTrajData* info      = &planner_manager_->local_data_;
  info->start_time_        = ros::Time::now();
  info->position_traj_     = NonUniformBspline(ctrl_pts, 3, dt);
  info->velocity_traj_     = info->position_traj_.getDerivative();
  info->acceleration_traj_ = info->velocity_traj_.getDerivative();
  info->start_pos_         = info->position_traj_.evaluateDeBoorT(0.0);
  info->duration_          = info->position_traj_.getTimeSum();
  info->traj_id_ += 1;

  // Yaw giu nguyen huong hien tai
  Eigen::Vector3d rot_x   = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
  int             seg_num = max(1, int(ceil(info->duration_ / 0.3)));
  Eigen::MatrixXd yaw     = Eigen::MatrixXd::Constant(seg_num + 3, 1, atan2(rot_x(1), rot_x(0)));
  info->yaw_traj_.setUniformBspline(yaw, 3, info->duration_ / seg_num);
  info->yawdot_traj_    = info->yaw_traj_.getDerivative();
  info->yawdotdot_traj_ = info->yawdot_traj_.getDerivative();

  publishLocalTraj();
  visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                              Eigen::Vector4d(1, 0, 0, 1));
  return true;
}

// KinoReplanFSM::
// [Luan van - M3] Can cat canh neu pha cat canh duoc bat va UAV dang thap hon do cao cat canh.
bool KinoReplanFSM::needTakeoff() {
  return takeoff_height_ > 0.0 && odom_pos_(2) < takeoff_height_ - 0.15;
}

}  // namespace fast_planner
