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



#include <plan_env/obj_predictor.h>
#include <string>
#include <limits>

namespace fast_planner {
/* ============================== obj history_ ============================== */

int ObjHistory::queue_size_;
int ObjHistory::skip_num_;
ros::Time ObjHistory::global_start_time_;

void ObjHistory::init(int id) {
  clear();
  skip_ = 0;
  obj_idx_ = id;
}

void ObjHistory::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  ++skip_;
  if (skip_ < ObjHistory::skip_num_) return;

  Eigen::Vector4d pos_t;
  pos_t(0) = msg->pose.position.x, pos_t(1) = msg->pose.position.y, pos_t(2) = msg->pose.position.z;
  pos_t(3) = (ros::Time::now() - ObjHistory::global_start_time_).toSec();

  history_.push_back(pos_t);
  // cout << "idx: " << obj_idx_ << "pos_t: " << pos_t.transpose() << endl;

  if (history_.size() > queue_size_) history_.pop_front();

  skip_ = 0;
}

// ObjHistory::
/* ============================== obj predictor ==============================
 */
ObjPredictor::ObjPredictor(/* args */) {
}

ObjPredictor::ObjPredictor(ros::NodeHandle& node) {
  this->node_handle_ = node;
}

ObjPredictor::~ObjPredictor() {
}

void ObjPredictor::init() {
  // [Luan van - M4] ba bien static cua ObjHistory truoc day khong duoc gan o dau ca:
  // queue_size_ = 0 lam lich su luon rong, global_start_time_ = 0 lam t ~ 1.7e9 s
  ObjHistory::global_start_time_ = ros::Time::now();
  node_handle_.param("prediction/queue_size", ObjHistory::queue_size_, 40);
  node_handle_.param("prediction/skip_num", ObjHistory::skip_num_, 5);

  /* get param */
  node_handle_.param("prediction/obj_num", obj_num_, 5);
  // [Luan van - M4] phat vi tri du doan tai (now + eval_horizon) de do sai so du doan
  node_handle_.param("prediction/eval_horizon", eval_horizon_, 1.0);
  pred_pub_ = node_handle_.advertise<geometry_msgs::PoseArray>("/dynamic/prediction", 10);
  node_handle_.param("prediction/lambda", lambda_, 1.0);
  node_handle_.param("prediction/predict_rate", predict_rate_, 1.0);

  predict_trajs_.reset(new vector<PolynomialPrediction>);
  predict_trajs_->resize(obj_num_);

  obj_scale_.reset(new vector<Eigen::Vector3d>);
  obj_scale_->resize(obj_num_);
  for (auto& sc : *obj_scale_) sc.setZero();  // [Luan van - M4] Eigen khong tu gan 0
  scale_init_.resize(obj_num_);
  for (int i = 0; i < obj_num_; i++)
    scale_init_[i] = false;

  /* subscribe to pose */
  for (int i = 0; i < obj_num_; i++) {
    shared_ptr<ObjHistory> obj_his(new ObjHistory);

    obj_his->init(i);
    obj_histories_.push_back(obj_his);

    ros::Subscriber pose_sub = node_handle_.subscribe<geometry_msgs::PoseStamped>(
        "/dynamic/pose_" + std::to_string(i), 10, &ObjHistory::poseCallback, obj_his.get());

    pose_subs_.push_back(pose_sub);
  }

  marker_sub_ = node_handle_.subscribe<visualization_msgs::Marker>("/dynamic/obj", 10,
                                                                   &ObjPredictor::markerCallback, this);

  /* update prediction */
  predict_timer_ =
      node_handle_.createTimer(ros::Duration(1 / predict_rate_), &ObjPredictor::predictCallback, this);
}

ObjPrediction ObjPredictor::getPredictionTraj() {
  return this->predict_trajs_;
}

ObjScale ObjPredictor::getObjScale() {
  return this->obj_scale_;
}

void ObjPredictor::predictPolyFit() {
  /* iterate all obj */
  for (int i = 0; i < obj_num_; i++) {
    /* ---------- write A and b ---------- */
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> temp;
    Eigen::Matrix<double, 6, 1> bm[3];  // poly coefficent
    vector<Eigen::Matrix<double, 6, 1>> pm(3);

    A.setZero();
    for (int i = 0; i < 3; ++i)
      bm[i].setZero();

    /* ---------- estimation error ---------- */
    list<Eigen::Vector4d> his;
    obj_histories_[i]->getHistory(his);
    for (list<Eigen::Vector4d>::iterator it = his.begin(); it != his.end(); ++it) {
      Eigen::Vector3d qi = (*it).head(3);
      double ti = (*it)(3);

      /* A */
      temp << 1.0, ti, pow(ti, 2), pow(ti, 3), pow(ti, 4), pow(ti, 5);
      for (int j = 0; j < 6; ++j)
        A.row(j) += 2.0 * pow(ti, j) * temp.transpose();

      /* b */
      for (int dim = 0; dim < 3; ++dim)
        bm[dim] += 2.0 * qi(dim) * temp;
    }

    /* ---------- acceleration regulator ---------- */
    double t1 = his.front()(3);
    double t2 = his.back()(3);

    temp << 0.0, 0.0, 2 * t1 - 2 * t2, 3 * pow(t1, 2) - 3 * pow(t2, 2), 4 * pow(t1, 3) - 4 * pow(t2, 3),
        5 * pow(t1, 4) - 5 * pow(t2, 4);
    A.row(2) += -4 * lambda_ * temp.transpose();

    temp << 0.0, 0.0, pow(t1, 2) - pow(t2, 2), 2 * pow(t1, 3) - 2 * pow(t2, 3),
        3 * pow(t1, 4) - 3 * pow(t2, 4), 4 * pow(t1, 5) - 4 * pow(t2, 5);
    A.row(3) += -12 * lambda_ * temp.transpose();

    temp << 0.0, 0.0, 20 * pow(t1, 3) - 20 * pow(t2, 3), 45 * pow(t1, 4) - 45 * pow(t2, 4),
        72 * pow(t1, 5) - 72 * pow(t2, 5), 100 * pow(t1, 6) - 100 * pow(t2, 6);
    A.row(4) += -4.0 / 5.0 * lambda_ * temp.transpose();

    temp << 0.0, 0.0, 35 * pow(t1, 4) - 35 * pow(t2, 4), 84 * pow(t1, 5) - 84 * pow(t2, 5),
        140 * pow(t1, 6) - 140 * pow(t2, 6), 200 * pow(t1, 7) - 200 * pow(t2, 7);
    A.row(5) += -4.0 / 7.0 * lambda_ * temp.transpose();

    /* ---------- solve ---------- */
    for (int j = 0; j < 3; j++) {
      pm[j] = A.colPivHouseholderQr().solve(bm[j]);
    }

    /* ---------- update prediction container ---------- */
    predict_trajs_->at(i).setPolynomial(pm);
    predict_trajs_->at(i).setTime(t1, t2);
  }
}

void ObjPredictor::predictCallback(const ros::TimerEvent& e) {
  // predictPolyFit();
  predictConstVel();

  // [Luan van - M4] header.stamp = thoi diem ma du doan huong toi (now + eval_horizon);
  // vat can chua co du doan thi de NaN
  geometry_msgs::PoseArray pa;
  ros::Time now = ros::Time::now();
  pa.header.frame_id = "world";
  pa.header.stamp = now + ros::Duration(eval_horizon_);
  double t = (now - ObjHistory::global_start_time_).toSec() + eval_horizon_;
  for (int i = 0; i < obj_num_; i++) {
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;
    if (predict_trajs_->at(i).valid()) {
      Eigen::Vector3d q = predict_trajs_->at(i).evaluateConstVel(t);
      p.position.x = q(0), p.position.y = q(1), p.position.z = q(2);
    } else {
      p.position.x = p.position.y = p.position.z = std::numeric_limits<double>::quiet_NaN();
    }
    pa.poses.push_back(p);
  }
  pred_pub_.publish(pa);
}

void ObjPredictor::markerCallback(const visualization_msgs::MarkerConstPtr& msg) {
  int idx = msg->id;
  if (idx < 0 || idx >= obj_num_) return;  // [Luan van - M4] obj_generator sinh nhieu hon obj_num
  (*obj_scale_)[idx](0) = msg->scale.x;
  (*obj_scale_)[idx](1) = msg->scale.y;
  (*obj_scale_)[idx](2) = msg->scale.z;

  scale_init_[idx] = true;

  int finish_num = 0;
  for (int i = 0; i < obj_num_; i++) {
    if (scale_init_[i]) finish_num++;
  }

  if (finish_num == obj_num_) {
    marker_sub_.shutdown();
  }
}

void ObjPredictor::predictConstVel() {
  for (int i = 0; i < obj_num_; i++) {
    /* ---------- get the last two point ---------- */
    list<Eigen::Vector4d> his;
    obj_histories_[i]->getHistory(his);
    // [Luan van - M4] can it nhat 2 mau moi uoc luong duoc van toc; truoc day
    // lui iterator qua dau list rong -> hanh vi khong xac dinh
    if (his.size() < 2) continue;
    list<Eigen::Vector4d>::iterator list_it = his.end();

    /* ---------- test iteration ---------- */
    // cout << "----------------------------" << endl;
    // for (auto v4d : his)
    //   cout << "v4d: " << v4d.transpose() << endl;

    Eigen::Vector3d q1, q2;
    double t1, t2;

    --list_it;
    q2 = (*list_it).head(3);
    t2 = (*list_it)(3);

    --list_it;
    q1 = (*list_it).head(3);
    t1 = (*list_it)(3);

    if (t2 - t1 < 1e-6) continue;  // [Luan van - M4] hai mau trung thoi diem -> ma tran suy bien
    Eigen::Matrix<double, 2, 3> p01, q12;
    q12.row(0) = q1.transpose();
    q12.row(1) = q2.transpose();

    Eigen::Matrix<double, 2, 2> At12;
    At12 << 1, t1, 1, t2;

    p01 = At12.inverse() * q12;

    vector<Eigen::Matrix<double, 6, 1>> polys(3);
    for (int j = 0; j < 3; ++j) {
      polys[j].setZero();
      polys[j].head(2) = p01.col(j);
    }

    predict_trajs_->at(i).setPolynomial(polys);
    predict_trajs_->at(i).setTime(t1, t2);
  }
}

// ObjPredictor::
}  // namespace fast_planner