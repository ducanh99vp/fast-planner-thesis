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



// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <std_msgs/Float64MultiArray.h>

namespace fast_planner {

// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */
  timing_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/benchmark/timing", 20);  
  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  /* [CD2] huong B: diem quy dao XA hon chan troi nay (giay) ma chi cham VET QUET
     (khong cham vat can that) thi KHONG bat lap lai ke hoach. < 0 = tat. */
  nh.param("manager/dyn_check_horizon", pp_.dyn_check_horizon_, -1.0);
  nh.param("manager/guide_ignore_dyn", guide_ignore_dyn_, false);   // [CD2 21/10]
  nh.param("manager/dyn_cost", pp_.dyn_cost_, false);                  // [CD2 P3]
  nh.param("manager/dyn_check_time", pp_.dyn_check_time_, false);      // [CD2 P3]
  nh.param("manager/dyn_check_margin", pp_.dyn_check_margin_, 0.2);
  nh.param("manager/dyn_check_tmax", pp_.dyn_check_tmax_, 1.5);
  nh.param("manager/dyn_static_min", pp_.dyn_static_min_, 0.05);
  nh.param("manager/dyn_post_margin", pp_.dyn_post_margin_, -1.0);    // [CD2 06/11]
  nh.param("manager/dyn_post_tmax", pp_.dyn_post_tmax_, 3.0);
  nh.param("manager/dyn_post_static_min", pp_.dyn_post_static_min_, -1.0);   // [CD2 07/11]
  nh.param("manager/dyn_post_wall", pp_.dyn_post_wall_, -1.0);               // [CD2 09/11]
  nh.param("manager/dyn_post_tol", pp_.dyn_post_tol_, 0.0);                  // [CD2 09/11]
  nh.param("manager/dyn_margin_em", pp_.dyn_margin_em_, -1.0);               // [CD2 10/11]
  nh.param("manager/dyn_dv_thresh", pp_.dyn_dv_thresh_, -1.0);               // [CD2 13/11] huong 4
  nh.param("manager/dyn_dv_range", pp_.dyn_dv_range_, 0.5);
  nh.param("manager/dyn_dv_T", pp_.dyn_dv_T_, 3.0);
  nh.param("manager/dyn_slow_vel", pp_.dyn_slow_vel_, -1.0);                 // [CD2 13/11] huong 5
  nh.param("manager/dyn_slow_range", pp_.dyn_slow_range_, 1.0);
  nh.param("manager/dyn_slow_T", pp_.dyn_slow_T_, 2.0);
  nh.param("manager/yaw_mode", pp_.yaw_mode_, 0);
  nh.param("manager/yaw_hold_dist", pp_.yaw_hold_dist_, 1.0);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }
}

void FastPlannerManager::publishTiming(double t_fe_ms, double t_be_ms, bool ok) {
  std_msgs::Float64MultiArray m;
  m.data.push_back(t_fe_ms);
  m.data.push_back(t_be_ms);
  m.data.push_back(ok ? 1.0 : 0.0);
  timing_pub_.publish(m);
}

bool FastPlannerManager::planGlobalPath(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                        vector<Eigen::Vector3d>& path) {   // [CD1]
  t_global_ms_ = 0.0;
  if (!geo_path_finder_) return false;
  ros::Time t1 = ros::Time::now();
  geo_path_finder_->reset();
  geo_path_finder_->setTimeBase(ros::Time::now().toSec());   // [CD2 P3L]
  int status   = geo_path_finder_->search(start, end, false, -1.0);
  t_global_ms_ = (ros::Time::now() - t1).toSec() * 1000.0;
  if (status != Astar::REACH_END) return false;
  path = geo_path_finder_->getPath();
  if (geo_path_finder_->n_dyn_pen_ > 0) {   // [CD2 P3L] chan doan: duong toan cuc co ne lan dong khong
    Eigen::Vector3d p2 = path.back();
    double s = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      s += (path[i] - path[i - 1]).norm();
      if (s >= 2.0) { p2 = path[i]; break; }
    }
    cout << "[CD2 P3L] duong toan cuc: " << geo_path_finder_->n_dyn_pen_ << " buoc bi phat vat can dong, diem 2 m: ("
         << p2(0) << ", " << p2(1) << ")" << endl;
  }
  return path.size() >= 2;
}

/* [CD1] Duong toan cuc cu (tu diem thu from) con trong tren ban do da nho khong. */
bool FastPlannerManager::globalPathFree(const vector<Eigen::Vector3d>& path, size_t from) {
  if (!geo_path_finder_) return false;
  ros::Time t1 = ros::Time::now();
  bool free = true;
  for (size_t i = from; i < path.size() && free; ++i) free = geo_path_finder_->pointFree(path[i]);
  t_global_ms_ = (ros::Time::now() - t1).toSec() * 1000.0;
  return free;
}

/* [CD1] Tam nhin: doan thang a-b khong cat o chiem dung nao (lay mau 0.05 m). KHONG dung
   khoang ho: chi can biet b co bi tuong che khong; giu khoang cach la viec cua A* cuc bo va bo
   toi uu. (Dung khoang ho 0.2 m thi o cua 0.9 m moi diem sau cua deu "khuat", dich tam chi con
   cach 0.3 m -> j_rms 14, do 14/10.) */
bool FastPlannerManager::segmentFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  const int n = std::max(1, (int)std::ceil((b - a).norm() / 0.05));
  for (int i = 1; i <= n; ++i)
  {
    const Eigen::Vector3d p = a + (b - a) * (double(i) / n);
    if (edt_environment_->sdf_map_->getInflateOccupancy(p) == 1 && !(guide_ignore_dyn_ && inDynSweep(p))) return false;
  }
  return true;
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance, bool real_only) {

  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      /* [CD2] huong B. Vet quet chi dung cho dyn_sweep_T giay toi, nhung phep kiem
         nay soi toi ~6 m (~4 s). Diem UAV con lau moi toi ma "cham" vet quet thi
         khong phan anh nguy co that, chi lam lap lai ke hoach thua (do duoc: tuong
         quan so lan lap lai voi do dai duong +0.90). Vat can THAT thi van bat. */
      const bool chi_vet_quet = (real_only || (pp_.dyn_check_horizon_ > 0.0 && fut_t > pp_.dyn_check_horizon_)) &&
                                !edt_environment_->sdf_map_->nearRealOccupied(fut_pt, 0.15);
      if (!chi_vet_quet) {
        distance = radius;
        return false;
      }
    }
    /* [CD2 P3] Vat can dong theo thoi gian: diem quy dao tai (bay gio + fut_t) so voi vi tri du doan cua
       vat can TAI DUNG thoi diem do; chi tin du doan toi dyn_check_tmax_. */
    if (pp_.dyn_check_time_ && !real_only && fut_t <= pp_.dyn_check_tmax_ &&
        edt_environment_->sdf_map_->dynPredMinDist(fut_pt, ros::Time::now().toSec() + fut_t) < pp_.dyn_check_margin_) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

/* [CD2 13/11] Huong 4: co vat can nao da doi van toc qua dyn_dv_thresh_ so voi luc lap ke hoach, ma theo du doan
   MOI quy dao dang bay se cach no < dyn_dv_range_ trong dyn_dv_T_ giay toi khong. */
bool FastPlannerManager::dynVelChanged(int& id_out, double& dv_out, double& d_out) {
  if (pp_.dyn_dv_thresh_ <= 0.0) return false;
  auto         map   = edt_environment_->sdf_map_;
  const double now   = ros::Time::now().toSec();
  const double t_now = now - local_data_.start_time_.toSec();
  double       tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  for (int i = 0; i < map->dynPredNum(); ++i) {
    int             id;
    Eigen::Vector3d v;
    if (!map->dynPredVel(i, id, v)) continue;
    auto it = plan_dyn_vel_.find(id);
    if (it == plan_dyn_vel_.end()) continue;
    const double dv = (v - it->second).norm();
    if (dv < pp_.dyn_dv_thresh_) continue;
    double d = 1e9;
    for (double ft = 0.0; ft <= pp_.dyn_dv_T_ && t_now + ft < local_data_.duration_; ft += 0.05)
      d = std::min(d, map->dynPredDist(i, local_data_.position_traj_.evaluateDeBoor(tm + t_now + ft), now + ft));
    if (d < pp_.dyn_dv_range_) {
      id_out = id, dv_out = dv, d_out = d;
      return true;
    }
  }
  return false;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  /* [CD2 Indoor] Ban goc gan moc = luc BAT DAU lap ke hoach. Neu lap ke hoach mat 0.8 s,
     traj_server danh gia quy dao moi o t = 0.8 s -> lenh vi tri nhay ~1.2 m -> nghieng 75 do. */
  /* [CD2 16/10] Chi gan moc vao local_data_ khi lap ke hoach THANH CONG. Neu that bai, local_data_
     van mo ta quy dao cu ma traj_server dang bay; truoc day moc bi doi ngay tu dau ham nen moi
     phep tinh sau that bai (t = now - start_time) deu lech. */
  const ros::Time new_start_time = desired_start_time_.isZero() ? ros::Time::now() : desired_start_time_;
  desired_start_time_            = ros::Time(0);
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();

  kino_path_finder_->reset();
  kino_path_finder_->setTimeBase(new_start_time.toSec());   // [CD2 P3] t = 0 cua A* = luc quy dao moi bat dau

  /* [CD2 13/11] Huong 5: sap giao cat vat can dong -> lap ke hoach o toc do thap (them thoi gian phan ung khi
     vat can doi toc do). Khong ap dung khi khan cap: luc do A* can moi lua chon. */
  bool cham = false;
  if (pp_.dyn_slow_vel_ > 0.0 && !khan_cap_) {
    double d = 1e9;
    for (double t = 0.0; t <= pp_.dyn_slow_T_ + 1e-6; t += 0.1)
      d = std::min(d, edt_environment_->sdf_map_->dynPredMinDist(start_pt + start_vel * t, new_start_time.toSec() + t));
    cham = d < pp_.dyn_slow_range_;
    if (cham)
      cout << "[CD2 cham] sap giao cat (bay thang se cach vat can dong " << d << " m trong " << pp_.dyn_slow_T_
           << " s) -> toc do toi da " << pp_.dyn_slow_vel_ << " m/s" << endl;
  }
  kino_path_finder_->setSlowVel(cham ? pp_.dyn_slow_vel_ : -1.0);

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    /* [CD2 10/11] Khan cap (quy dao cu sap cham): bao that bai thi FSM bay tiep quy dao cu vao vat can
       (OUT 18 f6: 15 lan that bai, -0.11 m). Noi bien vat can dong cua A* tung 0.1 m toi dyn_margin_em_;
       duong tim duoc la diem dau cho bo toi uu (f_d) va P3H day ra xa them. */
    if (status == KinodynamicAstar::NO_PATH && khan_cap_ && pp_.dyn_margin_em_ > 0.0) {
      const double m0 = kino_path_finder_->getDynMargin();
      for (double m = m0 - 0.1; m >= pp_.dyn_margin_em_ - 1e-6; m -= 0.1) {
        kino_path_finder_->setDynMargin(m);
        kino_path_finder_->reset();
        status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
        cout << "[CD2 khan] A* bien " << m << " m: " << (status == KinodynamicAstar::NO_PATH ? "that bai" : "co duong")
             << endl;
        if (status != KinodynamicAstar::NO_PATH) break;
      }
      kino_path_finder_->setDynMargin(m0);
    }

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      /* [CD2] chan doan: diem xuat phat co nam TRONG vung cam khong? Neu co thi
         moi buoc di dau tien deu cham vung cam va bi loai -> "open set empty". */
      cout << "[CD2] tim duong that bai: diem xuat phat bi chiem = "
           << edt_environment_->sdf_map_->getInflateOccupancy(start_pt)
           << ", khoang cach toi vat can gan nhat = "
           << edt_environment_->sdf_map_->getDistance(start_pt) << " m" << endl;
      publishTiming((ros::Time::now() - t1).toSec() * 1000.0 + t_global_ms_, 0.0, false);   // [CD1] + A* toan cuc
      t_global_ms_ = 0.0;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  t_search = (ros::Time::now() - t1).toSec();

  // parameterize the path to bspline

  double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  // bspline trajectory optimization

  t1 = ros::Time::now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  const Eigen::MatrixXd ctrl_init = ctrl_pts;   // [CD2 P3] de giai lai khi f_d pham khoang ho tinh
  if (pp_.dyn_cost_) {
    cost_function |= BsplineOptimizer::DYNAMIC;
    bspline_optimizers_[0]->setDynStart(new_start_time.toSec());
    bspline_optimizers_[0]->setDynScale(1.0);
  }
  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);
  /* [CD2 P3] Chot M5: f_d la chi phi mem, co the day quy dao vao vat can TINH (M5: dam tuong 6/6 khi bi
     don). Neu quy dao sau toi uu pham khoang ho tinh (ESDF < dyn_static_min_) thi giai lai voi f_d
     x0.5, x0.25, x0 — buoc cuoi bo han f_d nen luon con nghiem an toan voi vat can tinh. */
  auto staticOk = [&](const Eigen::MatrixXd& cp) {
    NonUniformBspline b(cp, 3, ts);
    const double T = b.getTimeSum();
    for (double t = 0.0; t <= T; t += 0.05) {
      Eigen::Vector3d p = b.evaluateDeBoorT(t);
      if (edt_environment_->evaluateCoarseEDT(p, -1.0) < pp_.dyn_static_min_) return false;
    }
    return true;
  };
  if (pp_.dyn_cost_) {
    for (double sc : { 0.5, 0.25, 0.0 }) {
      if (staticOk(ctrl_pts)) break;
      cout << "[CD2 P3] f_d day quy dao vao vat can tinh -> giai lai voi f_d x" << sc << endl;
      bspline_optimizers_[0]->setDynScale(sc);
      ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_init, ts, cost_function, 1, 1);
    }
    bspline_optimizers_[0]->setDynScale(1.0);
  }

  t_opt = (ros::Time::now() - t1).toSec();

  // iterative time adjustment

  t1 = ros::Time::now();
  /* [CD2 06/11] Gian thoi gian cua ban goc, tach thanh ham de dung lai khi giai lai voi f_d manh hon. */
  auto adjustTime = [&](const Eigen::MatrixXd& cp, double& ratio) {
    NonUniformBspline b(cp, 3, ts);
    const double      to = b.getTimeSum();
    b.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
    bool feasible = b.checkFeasibility(false);

    int iter_num = 0;
    while (!feasible && ros::ok()) {

      feasible = b.reallocateTime();

      if (++iter_num >= 3) break;
    }
    ratio = b.getTimeSum() / to;
    return b;
  };
  double            ratio;
  NonUniformBspline pos = adjustTime(ctrl_pts, ratio);
  t_adjust              = (ros::Time::now() - t1).toSec();

  /* [CD2 06/11] Kiem CUNG ke hoach SAU khi gian thoi gian. f_d la chi phi mem (d0 0.5, lambda9 5 can bang o
     ~0.22-0.25 m) va gian thoi gian lam lech them 0-8 cm, nen ke hoach phat ra co the da sat vat can dong
     hon le cua chinh planner (04/11: ca 3 luot T3 < 0.267 m deu do ke hoach chi cach 0.21-0.25 m). Neu
     khoang ho theo du doan trong dyn_post_tmax_ giay dau < dyn_post_margin_ thi giai lai tu nghiem A* voi
     f_d x2, x4, x8 (bo ung vien pham khoang ho tinh). Khong ung vien nao dat thi giu ung vien xa vat can
     nhat: van khong te hon ke hoach ban dau. */
  if (pp_.dyn_cost_ && pp_.dyn_post_margin_ > 0.0) {
    t1            = ros::Time::now();
    auto dynClear = [&](NonUniformBspline& b) {
      const double T = std::min(b.getTimeSum(), pp_.dyn_post_tmax_);
      double       d = 1e9;
      for (double t = 0.0; t <= T; t += 0.02) {
        Eigen::Vector3d p = b.evaluateDeBoorT(t);
        d = std::min(d, edt_environment_->sdf_map_->dynPredMinDist(p, new_start_time.toSec() + t));
      }
      return d;
    };
    /* [CD2 07/11, sua 09/11] Nguong toi thieu toi TUONG cho ung vien giai lai. I3 hat 9 (f5): f_d x8 day ke
       hoach 0.5 m ve phia tuong, chinh ke hoach cach tuong that 0.263 m (< 0.267) ma van qua staticOk.
       ESDF luong tu theo o 0.1 m (= 0.1*sqrt(n)): 0.1 = o ngay sat vung phinh = tuong that 0.2-0.4 m;
       0.2 = >= 0.3 m. Nen staticOk ">= 0.05" chi chan vung phinh, va nguong 0.15 = cach vung phinh >= 2 o.
       So TUNG THOI DIEM tren tham so thoi gian chung ts (truoc gian thoi gian, cung xuat phat ctrl_init):
       ESDF_ung_vien(t) >= min(ESDF_ban_dau(t), dyn_post_static_min_) -> cho nao ke hoach ban dau cung sat
       (qua cua, hanh lang) thi van cho, cho rong thi khong duoc keo vao tuong. (08/11: so min CA quy dao bi
       vo hieu vi ke hoach ban dau hau nhu luon co mot diem 0.1.)
       [09/11] dyn_post_wall_ > 0: do bang rawObstacleDist (m toi mat o chiem THAT) thay ESDF luong tu -
       09/11 nguong ESDF 0.15 bo ca ung vien that ra cach tuong ~0.35 m (I4D-R1: dong tut 0.35 -> 0.28). */
    const bool   do_that = pp_.dyn_post_wall_ > 0.0;
    const double nguong  = do_that ? pp_.dyn_post_wall_ : pp_.dyn_post_static_min_;
    auto staticProfile = [&](const Eigen::MatrixXd& cp) {
      NonUniformBspline b(cp, 3, ts);
      const double      T = b.getTimeSum();
      vector<double>    s;
      for (double t = 0.0; t <= T; t += 0.02) {
        Eigen::Vector3d p = b.evaluateDeBoorT(t);
        s.push_back(do_that ? edt_environment_->sdf_map_->rawObstacleDist(p, nguong + 0.1)
                            : edt_environment_->evaluateCoarseEDT(p, -1.0));
      }
      return s;
    };
    auto minOf = [](const vector<double>& v) { return v.empty() ? -1.0 : *std::min_element(v.begin(), v.end()); };
    double d_best = dynClear(pos);
    if (d_best < pp_.dyn_post_margin_) {
      const double   d_dau     = d_best;
      double         sc_best   = 1.0;
      const bool     co_nguong = nguong > 0.0;
      vector<double> s0;
      if (co_nguong) s0 = staticProfile(ctrl_pts);
      double s_best = minOf(s0);
      for (double sc : { 2.0, 4.0, 8.0 }) {
        bspline_optimizers_[0]->setDynScale(sc);
        Eigen::MatrixXd cp = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_init, ts, cost_function, 1, 1);
        if (!staticOk(cp)) {
          cout << "[CD2 P3H] f_d x" << sc << " pham khoang ho tinh, bo" << endl;
          continue;
        }
        vector<double> s1;
        if (co_nguong) {
          s1      = staticProfile(cp);
          int bad = -1;
          for (size_t i = 0; i < std::min(s0.size(), s1.size()); ++i)
            if (s1[i] < std::min(s0[i] - pp_.dyn_post_tol_, nguong) - 1e-9) {
              bad = int(i);
              break;
            }
          if (bad >= 0) {
            cout << "[CD2 P3H] f_d x" << sc << " sat tuong luc t=" << 0.02 * bad << " s (" << (do_that ? "that " : "ESDF ")
                 << s1[bad] << " < min(ban dau " << s0[bad] << " - " << pp_.dyn_post_tol_ << ", " << nguong << ")), bo"
                 << endl;
            continue;
          }
        }
        double            r;
        NonUniformBspline b = adjustTime(cp, r);
        const double      d = dynClear(b);
        if (d > d_best) d_best = d, pos = b, ratio = r, sc_best = sc, s_best = minOf(s1);
        if (d_best >= pp_.dyn_post_margin_) break;
      }
      bspline_optimizers_[0]->setDynScale(1.0);
      cout << "[CD2 P3H] ke hoach cach vat can dong " << d_dau << " m < " << pp_.dyn_post_margin_
           << " -> giai lai, chon f_d x" << sc_best << ": " << d_best << " m";
      if (co_nguong) cout << " | " << (do_that ? "tuong that" : "ESDF tuong") << " nho nhat " << minOf(s0) << " -> " << s_best;
      cout << endl;
    }
    t_opt += (ros::Time::now() - t1).toSec();   // giai lai tinh vao thoi gian toi uu (t_be)
  }

  cout << "[kino replan]: Reallocate ratio: " << ratio << endl;
  if (ratio > 3.0) ROS_ERROR("reallocate error.");

  // save planned results

  local_data_.start_time_    = new_start_time;   // [CD2 16/10]
  local_data_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  /* [CD2 13/11] Huong 4: ghi van toc tung vat can ma ke hoach nay da dung */
  if (pp_.dyn_dv_thresh_ > 0.0) {
    plan_dyn_vel_.clear();
    int             id;
    Eigen::Vector3d v;
    for (int i = 0; i < edt_environment_->sdf_map_->dynPredNum(); ++i)
      if (edt_environment_->sdf_map_->dynPredVel(i, id, v)) plan_dyn_vel_[id] = v;
  }

  publishTiming(t_search * 1000.0 + t_global_ms_, t_opt * 1000.0, true);   // [CD1] + A* toan cuc
  t_global_ms_ = 0.0;

  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  // generate global reference trajectory

  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // insert intermediate points if too far
  vector<Eigen::Vector3d> inter_points;
  const double            dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // write position matrix
  int             pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  time(0) *= 2.0;
  time(0) = max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = max(1.0, time(time.rows() - 1));

  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory

  double            dt, duration;
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_    = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  ros::Time t1, t2;

  /* truncate a new local segment for replanning */
  ros::Time time_now = ros::Time::now();
  double    t_now    = (time_now - global_data_.global_start_time_).toSec();
  double    local_traj_dt, local_traj_duration;
  double    time_inc = 0.0;

  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      ROS_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr>            graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
                               raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;
      selectBestTraj(best_traj);
      refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // sort by jerk
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
       [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
                                    << " seconds, time change is: " << time_inc);
}

void FastPlannerManager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int    prev_num    = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int    seg_num     = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  ratio = min(1.01, ratio);
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt              = duration / double(seg_num);
  time_inc        = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) {
  ros::Time t1;
  double    tm1, tm2, tm3;

  t1 = ros::Time::now();

  // parameterize B-spline according to the length of guide path
  int             seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double          dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // discretize the guide path and align it with B-spline control points
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (guide_pt.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("what guide");

  tm1 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // first phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (ros::Time::now() - t1).toSec();
  ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double& dt, double& duration) {
  /* get the sample points local traj within radius */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  /* parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
                                            vector<Eigen::Vector3d>& colli_end,
                                            vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts) {
  bool               last_safe = true, safe;
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int    sn = ceil((t_s - t_m) / dt);
  dt        = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    /* [CD2] Che do 2: UAV gan dung yen (dang phanh, nhuong vat can) thi huong pd chi
       la nhieu cua mot doan quy dao rat ngan, xoay toi 180 do trong vai phan giay.
       Giu nguyen huong mui thay vi bam theo no. */
    const bool giu_huong = pp_.yaw_mode_ >= 2 && pd.head<2>().norm() < pp_.yaw_hold_dist_;
    if (pd.norm() > 1e-6 && !giu_huong) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else if (pp_.yaw_mode_ >= 1) {
      waypt = Eigen::Vector3d(last_yaw, 0.0, 0.0);  // ban goc: waypts.back() vo khi i = 0
    } else {
      waypt = waypts.back();
    }
    /* [CD2] Loi QUAN GOC cua ban goc: last_yaw khong bao gio duoc cap nhat, nen moi
       diem moc bi quy ve [-pi, pi] quanh goc XUAT PHAT thay vi quanh diem lien truoc.
       Khi huong bay vuot qua phia doi dien huong xuat phat, hai diem moc canh nhau
       lech ~2*pi va bo toi uu noi chung bang mot VONG XOAY TRON (do duoc: 490-560 do
       trong ~3 s, o 4/4 luot xoay manh). */
    if (pp_.yaw_mode_ >= 1) last_yaw = waypt(0);
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  // [CD2] che do 2: van toc cuoi ~0 (toi dich, dung lai) thi atan2 chi la nhieu
  /* [CD2 30/10] last_yaw da CONG DON (khong nam trong [-pi, pi]); calcNextYaw chi dung voi goc trong
     [-pi, pi] nen cong them (k - 1) vong khi |last_yaw| > 3*pi -> chay tay qua lai 40 phut, goc cuoi
     quy dao 257.6 -> 509.4 rad, UAV xoay 40 vong truoc dich, nghieng 76 do. Giu mui thi dung thang. */
  if (pp_.yaw_mode_ >= 2 && end_v.head<2>().norm() < 0.3)
    end_yaw(0) = last_yaw;
  else
    calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
