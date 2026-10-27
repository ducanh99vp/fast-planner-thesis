#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// #include <path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "plan_env/edt_environment.h"

namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstar {
 private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;
  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
  // shared_ptr<SDFMap> sdf_map;
  EDTEnvironment::Ptr edt_environment_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  /* [CD2] Bong bong thoat. Khi diem xuat phat NAM TRONG vung cam (vet quet vat
     can dong lan toi UAV), moi buoc di dau tien deu cham vung cam va bi loai ->
     "open set empty, no path" hang tram lan lien tiep. Trong ban kinh
     start_escape_ quanh diem xuat phat thi bo qua kiem va chach, de A* tim duoc
     duong DI RA. Chi bat khi diem xuat phat thuc su bi chiem. */
  double start_escape_;
  double start_escape_default_ = 0.0;   // [CD2] gia tri tham so, de tra lai sau khi ne
  bool   escape_active_;
  /* [CD2 Indoor] Gioi han thoi gian moi lan tim (s); <= 0 = khong gioi han (ban goc).
     Trong nha, khi vet quet chan ngang hanh lang, A* vet can khong gian toi 0.5-0.8 s. */
  double max_search_time_;
  Eigen::Vector3d search_start_;
  bool occupiedForSearch(const Eigen::Vector3d& pos);
  /* [CD2 21/10] Ghi log chan doan: search/debug_dir khac rong -> moi lan tim ghi mot tep: trang
     thai xuat phat, ket qua, duong tim duoc, MOI nut da duyet kem so buoc bi loai theo ly do,
     va lat cat ban do quanh diem xuat phat. Rong = tat (ban goc). */
  /* [CD2 21/10] Dai do cao cho A* cuc bo (mac dinh tat). Noi ra cho chua do cao xuat phat, neu khong
     UAV dang o ngoai dai thi moi nut deu bi loai. Chan bay qua dau vat can tru + bot khong gian tim. */
  double z_min_ = -1e5, z_max_ = 1e5, zlo_ = -1e5, zhi_ = 1e5;
  /* [CD2 P3] Kiem vat can dong THEO THOI GIAN: nut o thoi diem t (tinh tu diem xuat phat) bi loai neu
     cach vat can du doan TAI t duoi dyn_margin_. Chi xet t <= dyn_tmax_. t_base_ = thoi gian tuyet doi
     cua diem xuat phat (planner dat truoc moi lan tim). */
  bool   dyn_time_ = false;
  double dyn_margin_ = 0.3, dyn_tmax_ = 3.0, t_base_ = 0.0, t_node0_ = 0.0, shot_t0_ = 0.0;
  bool   dynHit(const Eigen::Vector3d& pos, double t_rel);
  bool   dyn_escape_ = false;   // [CD2 P3] bong bong chi bat khi DIEM XUAT PHAT da trong bien vat can
  double dyn_start_d_ = 1e9;    // [CD2 P3 26/10] khoang cach diem xuat phat toi vat can du doan
  double dyn_escape_floor_ = 0.0;   // [CD2 P3 26/10] trong bong bong khong lai gan hon min(dyn_start_d_, nguong nay)
  std::string debug_dir_;
  int debug_count_ = 0;
  struct DbgNode {
    Eigen::Vector3d p, v;
    double g, f, t;
    int add, r_close, r_vel, r_same, r_dyn, r_static;
  };
  std::vector<DbgNode> dbg_nodes_;
  int searchImpl(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                 Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start);
  void dumpSearch(int ket_qua, const Eigen::Vector3d& sp, const Eigen::Vector3d& sv,
                  const Eigen::Vector3d& sa, const Eigen::Vector3d& ep, bool init);
  double tie_breaker_;
  bool optimistic_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  double time_origin_;

  /* helper */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  vector<double> cubic(double a, double b, double c, double d);
  vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                    double tau);

 public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);

  void setEnvironment(const EDTEnvironment::Ptr& env);
  void setTimeBase(double t_abs) { t_base_ = t_abs; }   // [CD2 P3]
  /* [CD2] r > 0: bong bong thoat tam thoi max(r, tham so) (duong ne ngang dai hon 0.6 m);
     r <= 0: tra lai tham so. */
  void setStartEscape(double r) {
    start_escape_ = (r > 0.0 && start_escape_default_ > 0.0) ? std::max(r, start_escape_default_)
                                                             : start_escape_default_;
  }

  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif