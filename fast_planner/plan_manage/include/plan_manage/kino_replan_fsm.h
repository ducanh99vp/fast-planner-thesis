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



#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {

class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  ros::NodeHandle nh_;

public:
  Test(const int& v) {
    test_ = v;
  }
  Test(ros::NodeHandle& node) {
    nh_ = node;
  }
  ~Test() {
  }
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW, TAKEOFF };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double takeoff_height_;  // [Luan van - M3] <= 0: tat pha cat canh
  double goal_z_;   // [CD2] do cao dich; goc ep cung 1.0
  bool global_guide_;   // [CD1] dan duong toan cuc: lap ke hoach cuc bo toi diem tren duong toan cuc
  bool goal_retry_;     // [CD2] het quy dao ma con cach dich > 0.5 m thi lap lai, khong bo dich
  bool guide_dyn_soft_ = false;   // [CD2 P3L] duong toan cuc co chi phi mem vat can dong -> khong giu duong cu
  Eigen::Vector3d goal_true_;   // [CD2] dich nhan duoc; end_pt_ co the la dich TAM khi vet quet de len dich
  double dyn_dodge_;   // [CD2] > 0: ne ngang khoi vet quet phia truoc vat can, cach mep them dyn_dodge_ m
  bool dodge_traj_ = false;   // [CD2] quy dao dang bay la quy dao ne ngang
  double guide_dist_;   // [CD1] khoang cach (theo duong) tu xuat phat toi dich tam
  double guide_vel_;    // [CD1] toc do tai dich tam, theo huong duong toan cuc (0 = dung lai)
  vector<Eigen::Vector3d> guide_path_;   // [CD1] duong toan cuc dang theo
  double guide_hidden_r_;   // [CD1] dich tam KHUAT van duoc chon neu cach diem xuat phat <= ban kinh nay
  double guide_replan_t_;   // [CD1] lap lai khi quy dao con < guide_replan_t_ giay
  double replan_latency_;   // [CD2 Indoor] > 0: lap ke hoach tu trang thai sau ngan nay giay
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  bool takeoff_planned_;  // [Luan van - M3]
  /* [CD2 P3] Cat canh co canh chung: pha cat canh (M3) la quy dao thang dung co dinh, FSM khong kiem va cham.
     Bat takeoff_watch_: tu z >= takeoff_min_z_, neu du doan P3 thay vat can se cach cot cat canh duoi
     takeoff_margin_ trong thoi gian cat canh con lai + 1 s thi ket thuc cat canh som, giao cho planner. */
  bool   takeoff_watch_ = false;
  double takeoff_min_z_ = 0.4, takeoff_margin_ = 0.5;
  /* [CD2 P3 27/10] Doi dich TAM theo du doan: P3 khong ghi vat can dong vao ban do nen phep kiem dich goc
     (ESDF) khong bao gio thay vat can dong sap quet qua dich (I4-R2 hat 7: tru di ngang dich, UAV cho sat
     lan cua tru roi cham). goal_dyn_T_ > 0: UAV cach dich that < goal_dyn_range_ ma du doan thay vat can se
     cach dich < goal_dyn_occ_ trong goal_dyn_T_ giay toi -> dich tam GAN nhat vua trong tinh (ESDF > 0.5)
     vua cach moi vat can du doan >= goal_dyn_free_; tra lai dich that khi no cung dat goal_dyn_free_. */
  double goal_dyn_T_ = 0.0, goal_dyn_occ_ = 0.5, goal_dyn_free_ = 0.7, goal_dyn_range_ = 5.0;
  double goalDynClear(const Eigen::Vector3d& p);
  bool   findDynSafeGoal(const Eigen::Vector3d& gc, Eigen::Vector3d& goal);
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  bool callKinodynamicReplan();        // front-end and back-end method
  Eigen::Vector3d guideTarget(Eigen::Vector3d& target_vel);   // [CD1] dich tam tren duong toan cuc
  bool planTakeoffTraj();   // [Luan van - M3] quy dao cat canh thang dung, khong qua A*
  bool planStopTraj(const ros::Time& t0);   // [CD2 25/10] quy dao phanh dung tu trang thai ban giao
  bool stop_on_fail_ = false;   // [CD2 25/10] lap ke hoach that bai + quy dao cu sap cham -> phanh dung
  bool stopIfUnsafe(const ros::Time& t0);   // [CD2 25/10]
  void publishLocalTraj();  // [Luan van - M3] gui local_data_ cho traj_server
  bool needTakeoff();                                        // [Luan van - M3]
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  KinoReplanFSM(/* args */) {
  }
  ~KinoReplanFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif