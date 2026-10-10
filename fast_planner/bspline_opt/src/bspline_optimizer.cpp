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



#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
#include <plan_env/box_dist.h>  // [Luan van - M5] signedDistToBox
// using namespace std;

namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);
const int BsplineOptimizer::DYNAMIC     = (1 << 7);  // [Luan van - M5]
const int BsplineOptimizer::LENGTH      = (1 << 8);  // [Luan van - M6]


const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/lambda4", lambda4_, -1.0);
  nh.param("optimization/lambda5", lambda5_, -1.0);
  nh.param("optimization/lambda6", lambda6_, -1.0);
  nh.param("optimization/lambda7", lambda7_, -1.0);
  nh.param("optimization/lambda8", lambda8_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  /* [Luan van - M5] vat can dong. lambda9 = 0 lam f_d tro thanh vo hieu,
     tuc la quay ve dung hanh vi cua B0 — do la kieu hong an toan. */
  nh.param("optimization/lambda9", lambda9_, 0.0);
  /* [Luan van - M6] trong so do dai duong di. 0 = tat (hanh vi cu khong doi). */
  nh.param("optimization/lambda10", lambda10_, 0.0);
  nh.param("optimization/dist_dyn0", dist_dyn0_, 0.7);
  nh.param("optimization/t_dyn_max", t_dyn_max_, 2.0);
  /* [Luan van - M5] bien an toan tang tuyen tinh theo thoi diem cua diem dieu khien.
     0 = bien co dinh (hanh vi B1/B2). Sai so du doan do tren I1 tang ~0.5 m/s
     (trung binh) va ~1.2 m/s (p95). */
  nh.param("optimization/dyn_margin_rate", dyn_margin_rate_, 0.0);
  /* [Luan van - M6] Buoc lay mau DUONG CONG cho f_d. 0 = che do cu (kiem tai
     he so dieu khien), nen moi so lieu M3/M5/M6 giu nguyen khong doi. */
  nh.param("optimization/dyn_sample_dt", dyn_sample_dt_, 0.0);
  dyn_scale_   = 1.0;
  dyn_start_t_ = 0.0;
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/visib_min", visib_min_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";
  if (cost_function_ & LENGTH) cost_str += " length |";

  ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);
  g_dynamic_.resize(pt_num);
  g_length_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost);

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}
void BsplineOptimizer::setDynStartTime(const double& t) { dyn_start_t_ = t; }

void BsplineOptimizer::setDynScale(const double& s) { dyn_scale_ = s; }

/* [Luan van - M5] Chi phi vat can DONG.
 *
 * Diem dieu khien Q_i cua B-spline bac 3 nut deu anh huong manh nhat tai hoanh
 * do Greville t_i = (i - 1) * ts (kiem bang so: dinh cua ham co so N_{i,3} roi
 * dung vao do). Thoi diem tuyet doi de tra bo du doan la dyn_start_t_ + t_i.
 *
 * f = sum_i sum_k (d_ik - dist_dyn0_)^2  voi moi d_ik < dist_dyn0_,
 * trong do d_ik la khoang cach CO DAU tu Q_i toi hop du doan cua vat can k.
 * Cong don TAT CA vat can gan chu khong chi vat can gan nhat, de gradient
 * khong nhay khi thu tu cac vat can doi cho nhau.
 *
 * Khong dung EDTEnvironment::distToBox vi ham do tra ve 0 cho moi diem ben
 * trong hop -> gradient bang 0 dung luc UAV da lot vao trong.
 */

void BsplineOptimizer::calcDynamicCost(const vector<Eigen::Vector3d>& q, double& cost,
                                       vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  const int n_obs = edt_environment_->getDynObsNum();
  if (n_obs <= 0) return;

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  /* [Luan van - M6] CHE DO LAY MAU DUONG CONG.
   *
   * Vi sao can: vong lap cu hoi khoang cach tai HE SO q[i], moc Greville
   * t_i = (i-1)*dt. He so tu do dau tien la q[order_] voi moc 0.667 s, nen moi
   * xung dot trong 0.667 s dau KHONG sinh gradient nao — co che ne bi mu, chu
   * khong phai bi cam. Ngoai ra che do cu chi kiem 3 moc (0.667/1.0/1.333 s),
   * giua hai moc vat can dich duoc 0.27 m — gan bang nua kich thuoc hop cua no,
   * nen co the lot qua quy dao ma khong moc nao bat duoc.
   *
   * Cach lam: voi moi moc t = 0, sdt, 2*sdt, ... <= t_cap, tinh diem TREN
   * duong cong bang co so B-spline deu bac 3:
   *   s = t/dt,  a = floor(s),  u = s - a
   *   P(t) = b0*q[a] + b1*q[a+1] + b2*q[a+2] + b3*q[a+3]
   *   b0 = (1-u)^3/6                b1 = (3u^3 - 6u^2 + 4)/6
   *   b2 = (-3u^3 + 3u^2 + 3u+1)/6  b3 = u^3/6
   * Tai u=0 co [1/6, 4/6, 1/6, 0], dung bang P(0) = (q0 + 4q1 + q2)/6 — tuc
   * vi tri hien tai cua UAV. Tong co so = 1 (sai so 1.1e-16).
   *
   * Gradient phan ve 4 he so theo dung trong so co so (quy tac chuoi):
   *   d(cost)/dq[a+m] = 2*w_samp*(d - d0) * dd/dP * b[m]
   * He so bi khoa van nhan phan cua chung nhung combineCost chi doc tu order_
   * tro len, nen bi bo di dung nhu truoc. Ti le luc CON LAI tang dan theo t:
   *   0 s: 0%   0.2 s: 3.6%   0.333 s: 16.7%   0.5 s: 50%   0.667 s: 83.3%
   * — vach da cua che do cu tro thanh duong doc.
   *
   * w_samp = sdt/dt bien tong thanh xap xi tich phan theo thoi gian, cung don
   * vi voi che do cu. Khong co no, f_d lon gap 3.73 lan (do 16 moc thay vi 3)
   * nen lambda9 = 5.0 hoa ra nang nhu 18.7 — phep so A/B se bi nhieu boi trong
   * so, khong con tach duoc nguyen nhan. Co w_samp thi ti le con x1.12, va gan
   * nhu KHONG doi theo sdt (x1.07 / x1.12 / x1.19 cho 0.05 / 0.1 / 0.167 s),
   * nen doi buoc lay mau khong phai chinh lai lambda9.
   *
   * Kiem chung gradient bang sai phan trung tam: sai so 1.79e-09 / 207 phep.
   * Chi phi: 745 us cho 300 vong (4 vat can, sdt = 0.1 s) = 15% han muc 5 ms. */
  if (dyn_sample_dt_ > 0.0) {
    const double w_samp = dyn_sample_dt_ / bspline_interval_;
    const double t_cap =
        std::min(t_dyn_max_, double(int(q.size()) - 4) * bspline_interval_);
    if (t_cap < 0.0) return;
    const int n_samp = int(std::floor(t_cap / dyn_sample_dt_)) + 1;

    for (int si = 0; si < n_samp; ++si) {
      const double t = double(si) * dyn_sample_dt_;
      const double s = t / bspline_interval_;
      const int    a = int(std::floor(s));
      const double u = s - double(a);
      if (a < 0 || a + 3 >= int(q.size())) continue;

      const double u2 = u * u, u3 = u2 * u;
      double b[4];
      b[0] = (1.0 - 3.0 * u + 3.0 * u2 -       u3) / 6.0;
      b[1] = (4.0           - 6.0 * u2 + 3.0 * u3) / 6.0;
      b[2] = (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
      b[3] =                                   u3  / 6.0;

      const Eigen::Vector3d p =
          b[0] * q[a] + b[1] * q[a + 1] + b[2] * q[a + 2] + b[3] * q[a + 3];
      const double d0 = dist_dyn0_ + dyn_margin_rate_ * t;

      for (int k = 0; k < n_obs; k++) {
        Eigen::Vector3d center, half, dist_grad;
        if (!edt_environment_->getDynObsBox(k, dyn_start_t_ + t, center, half)) continue;

        double dist = signedDistToBox(p, center, half, dist_grad);
        if (dist < d0) {
          cost += w_samp * pow(dist - d0, 2);
          const Eigen::Vector3d g = 2.0 * w_samp * (dist - d0) * dist_grad;
          for (int m = 0; m < 4; ++m) gradient[a + m] += b[m] * g;
        }
      }
    }
    return;
  }

  for (int i = order_; i < end_idx; i++) {
    double t_i = (i - 1) * bspline_interval_;
    if (t_i > t_dyn_max_) break;  // xa hon chan troi tin cay cua du doan van toc deu
    /* [Luan van - M5] du doan cang xa cang kem tin -> doi khoang cach rong hon.
       d0 khong phu thuoc q[i] nen gradient giu nguyen dang. */
    const double d0 = dist_dyn0_ + dyn_margin_rate_ * t_i;

    for (int k = 0; k < n_obs; k++) {
      Eigen::Vector3d center, half, dist_grad;
      if (!edt_environment_->getDynObsBox(k, dyn_start_t_ + t_i, center, half)) continue;

      double dist = signedDistToBox(q[i], center, half, dist_grad);
      if (dist < d0) {
        cost += pow(dist - d0, 2);
        gradient[i] += 2.0 * (dist - d0) * dist_grad;
      }
    }
  }
}

/* [Luan van - M6] Chi phi DO DAI duong di, do bang do dai da tuyen cua cac diem
 * dieu khien: f_L = sum ||Q_{i+1} - Q_i||.
 *
 * Vi sao can: thoi luong quy dao da bi an dinh TRUOC khi toi uu hoa, va hien
 * khong co so hang nao phat do dai. Nen bo toi uu KHONG phan biet giua "lui lai
 * roi quay len", "lieng mot vong", va "truot sang ben" — ca ba cung thoi luong
 * va cung tranh duoc vat can. Do do no chon theo gradient cua f_d, ma gradient
 * do xuyen tam nen khi vat can lao thang vao mat thi huong duy nhat la LUI.
 * So hang nay pha vo the bat phan do: duong ngan nhat tranh duoc vat can la
 * truot sang ben, nen bo toi uu tu chon phuong an do.
 * Diem then chot: duong ngan hon van kha thi trong thoi luong co dinh, vi gioi
 * han van toc la gioi han TREN — bay cham hon khong vi pham gi.
 *
 * Gradient giai tich: voi doan e = Q_{i+1} - Q_i, d = ||e||, u = e/d thi
 *   d(f_L)/dQ_i = -u   va   d(f_L)/dQ_{i+1} = +u
 * Kiem chung bang sai phan trung tam: sai so lon nhat 1.72e-08 tren 13 phep thu.
 * Chan d < 1e-6 de khong chia cho 0 khi hai diem trung nhau. */
void BsplineOptimizer::calcLengthCost(const vector<Eigen::Vector3d>& q, double& cost,
                                      vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // bat dau tu order_ - 1 de tinh ca doan noi diem bi khoa voi diem tu do dau tien
  const int end_idx = (cost_function_ & ENDPOINT) ? int(q.size()) : int(q.size()) - order_;
  for (int i = order_ - 1; i + 1 < end_idx; ++i) {
    Eigen::Vector3d e = q[i + 1] - q[i];
    const double    d = e.norm();
    if (d < 1e-6) continue;
    cost += d;
    const Eigen::Vector3d u = e / d;
    gradient[i] -= u;
    gradient[i + 1] += u;
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; j++) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;
  double f_dynamic = 0.0;  // [Luan van - M5]
  double f_length  = 0.0;  // [Luan van - M6]

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  if ((cost_function_ & DYNAMIC) && dyn_scale_ > 0.0) {  // [Luan van - M5]
    const double w9 = lambda9_ * dyn_scale_;
    calcDynamicCost(g_q_, f_dynamic, g_dynamic_);
    f_combine += w9 * f_dynamic;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += w9 * g_dynamic_[i + order_](j);
  }
  if ((cost_function_ & LENGTH) && lambda10_ > 0.0) {  // [Luan van - M6]
    calcLengthCost(g_q_, f_length, g_length_);
    f_combine += lambda10_ * f_length;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda10_ * g_length_[i + order_](j);
  }

  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner