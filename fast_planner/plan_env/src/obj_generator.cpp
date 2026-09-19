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



#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <unordered_set>
#include <string>

#include <plan_env/linear_obj_model.hpp>
using namespace std;

int obj_num;
double _xy_size, _h_size, _vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;
// [Luan van - M4] vung rieng cho x, y (phong trong nha khong vuong), do cao tam co dinh, seed
double _x_size, _y_size, _z_center;
int _seed;

// [Luan van - M4] ban do tinh dang luoi voxel 0.1 m (bang bam) de vat can dong ne cot, tuong
const double VOX = 0.1;
unordered_set<long long> static_vox;
bool have_static_map = false;
ros::Subscriber map_sub;
double _static_margin;
double _height;  // [Luan van - M4] > 0: vat can cao tu san toi do cao nay (giong cot tinh)

long long voxKey(int x, int y, int z) {
  return ((long long)(x + 100000) << 40) | ((long long)(y + 100000) << 20) | (long long)(z + 100000);
}

void mapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (have_static_map) return;
  sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz)
    static_vox.insert(voxKey((int)floor(*ix / VOX), (int)floor(*iy / VOX), (int)floor(*iz / VOX)));
  have_static_map = !static_vox.empty();
  if (have_static_map) {
    map_sub.shutdown();
    ROS_INFO("[dynamic]: da nap ban do tinh, %lu voxel", static_vox.size());
  }
}

// Hop (tam pos, kich thuoc scale, song song truc) co cham voxel tinh nao khong.
// push = huong day hop ra xa vat can trong mat phang xy; bang 0 neu khong xac dinh duoc.
bool hitStatic(const Eigen::Vector3d& pos, const Eigen::Vector3d& scale, Eigen::Vector3d& push) {
  push.setZero();
  if (!have_static_map) return false;
  // theo z thu hep 0.15 m moi dau: hop cao tu san toi tran khong bi tinh la cham san, tran
  Eigen::Vector3d half = 0.5 * scale + Eigen::Vector3d(_static_margin, _static_margin, -0.15);
  Eigen::Vector3d lo = pos - half, hi = pos + half;
  bool hit = false;
  for (int x = (int)floor(lo(0) / VOX); x <= (int)floor(hi(0) / VOX); ++x)
    for (int y = (int)floor(lo(1) / VOX); y <= (int)floor(hi(1) / VOX); ++y)
      for (int z = (int)floor(lo(2) / VOX); z <= (int)floor(hi(2) / VOX); ++z)
        if (static_vox.count(voxKey(x, y, z))) {
          hit = true;
          push(0) += pos(0) - (x + 0.5) * VOX;
          push(1) += pos(1) - (y + 0.5) * VOX;
        }
  if (push.norm() > 1e-6) push.normalize();
  return hit;
}

ros::Publisher obj_pub;            // visualize marker
vector<ros::Publisher> pose_pubs;  // obj pose (from optitrack)
vector<LinearObjModel> obj_models;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_pos;
uniform_real_distribution<double> rand_pos_y;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_vel;
uniform_real_distribution<double> rand_acc_r;
uniform_real_distribution<double> rand_acc_t;
uniform_real_distribution<double> rand_acc_z;
uniform_real_distribution<double> rand_color;
uniform_real_distribution<double> rand_scale;
uniform_real_distribution<double> rand_yaw_dot;
uniform_real_distribution<double> rand_yaw;

ros::Time time_update, time_change;

void updateCallback(const ros::TimerEvent& e);
void visualizeObj(int id);

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obj");
  ros::NodeHandle node("~");

  /* ---------- initialize ---------- */
  node.param("obj_generator/obj_num", obj_num, 10);
  node.param("obj_generator/xy_size", _xy_size, 15.0);
  node.param("obj_generator/h_size", _h_size, 5.0);
  node.param("obj_generator/vel", _vel, 5.0);
  node.param("obj_generator/yaw_dot", _yaw_dot, 5.0);
  node.param("obj_generator/acc_r1", _acc_r1, 4.0);
  node.param("obj_generator/acc_r2", _acc_r2, 6.0);
  node.param("obj_generator/acc_z", _acc_z, 3.0);
  node.param("obj_generator/scale1", _scale1, 1.5);
  node.param("obj_generator/scale2", _scale2, 2.5);
  node.param("obj_generator/interval", _interval, 2.5);
  // [Luan van - M4] mac dinh giu nguyen hanh vi cu: x, y dung chung xy_size,
  // do cao tam ngau nhien, seed tu random_device (moi luot mot bo vat can khac)
  node.param("obj_generator/x_size", _x_size, _xy_size);
  node.param("obj_generator/y_size", _y_size, _xy_size);
  node.param("obj_generator/z_center", _z_center, -1.0);
  node.param("obj_generator/seed", _seed, -1);
  if (_seed >= 0) eng.seed(_seed);
  node.param("obj_generator/static_margin", _static_margin, 0.1);
  node.param("obj_generator/height", _height, -1.0);

  obj_pub = node.advertise<visualization_msgs::Marker>("/dynamic/obj", 10);
  for (int i = 0; i < obj_num; ++i) {
    ros::Publisher pose_pub =
        node.advertise<geometry_msgs::PoseStamped>("/dynamic/pose_" + to_string(i), 10);
    pose_pubs.push_back(pose_pub);
  }
  
  // [Luan van - M4] cho ban do tinh (toi da 5 s) truoc khi dat vat can. Khong co ban do
  // thi chay nhu ban goc: vat can chi phan xa o bien x_size, y_size
  map_sub = node.subscribe("/map_generator/global_cloud", 1, mapCallback);
  ros::Time t_wait = ros::Time::now();
  while (ros::ok() && !have_static_map && (ros::Time::now() - t_wait).toSec() < 5.0) {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  if (!have_static_map)
    ROS_WARN("[dynamic]: khong nhan duoc ban do tinh, vat can dong se xuyen qua vat can tinh");

  ros::Timer update_timer = node.createTimer(ros::Duration(1 / 30.0), updateCallback);
  cout << "[dynamic]: initialize with " + to_string(obj_num) << " moving obj." << endl;
  ros::Duration(1.0).sleep();

  rand_color = uniform_real_distribution<double>(0.0, 1.0);
  rand_pos = uniform_real_distribution<double>(-_x_size, _x_size);
  rand_pos_y = uniform_real_distribution<double>(-_y_size, _y_size);
  rand_h = uniform_real_distribution<double>(0.0, _h_size);
  rand_vel = uniform_real_distribution<double>(-_vel, _vel);
  rand_acc_t = uniform_real_distribution<double>(0.0, 6.28);
  rand_acc_r = uniform_real_distribution<double>(_acc_r1, _acc_r2);
  rand_acc_z = uniform_real_distribution<double>(-_acc_z, _acc_z);
  rand_scale = uniform_real_distribution<double>(_scale1, _scale2);
  rand_yaw = uniform_real_distribution<double>(0.0, 2 * 3.141592);
  rand_yaw_dot = uniform_real_distribution<double>(-_yaw_dot, _yaw_dot);

  /* ---------- give initial value of each obj ---------- */
  for (int i = 0; i < obj_num; ++i) {
    LinearObjModel model;
    Eigen::Vector3d scale(rand_scale(eng), 1.5 * rand_scale(eng), rand_scale(eng));
    if (_height > 0.0) scale(2) = _height;  // [Luan van - M4] cao bang vat can tinh
    // [Luan van - M4] boc lai vi tri neu hop de len vat can tinh (toi da 200 lan)
    Eigen::Vector3d pos, push;
    for (int k = 0; k < 200; ++k) {
      double pz = _height > 0.0 ? 0.5 * _height : (_z_center >= 0.0 ? _z_center : rand_h(eng));
      pos = Eigen::Vector3d(rand_pos(eng), rand_pos_y(eng), pz);
      if (!hitStatic(pos, scale, push)) break;
    }
    Eigen::Vector3d vel(rand_vel(eng), rand_vel(eng), 0.0);
    Eigen::Vector3d color(rand_color(eng), rand_color(eng), rand_color(eng));
    // [Luan van - M4] khong xoay (yaw_dot = 0) thi dat yaw = 0: hop song song truc toa do,
    // khop voi mo hinh hop cua planner (distToBox) va phep kiem va cham o tren
    double yaw = (_yaw_dot == 0.0) ? 0.0 : rand_yaw(eng);

    double yaw_dot = rand_yaw_dot(eng);

    double r, t, z;
    r = rand_acc_r(eng);
    t = rand_acc_t(eng);
    z = rand_acc_z(eng);
    Eigen::Vector3d acc(r * cos(t), r * sin(t), z);

    model.initialize(pos, vel, acc, yaw, yaw_dot, color, scale);
        // [Luan van - M4] bien z phai chua tam hop, neu khong update() se keo tam xuong
    double bz = std::max(_h_size, _height);
    model.setLimits(Eigen::Vector3d(_x_size, _y_size, bz), Eigen::Vector2d(0.0, _vel),
                    Eigen::Vector2d(0, 0));
    obj_models.push_back(model);
  }

  time_update = ros::Time::now();
  time_change = ros::Time::now();

  /* ---------- start loop ---------- */
  ros::spin();

  return 0;
}

void updateCallback(const ros::TimerEvent& e) {
  ros::Time time_now = ros::Time::now();

  /* ---------- change input ---------- */
  double dtc = (time_now - time_change).toSec();
  if (dtc > _interval) {
    for (int i = 0; i < obj_num; ++i) {
      /* ---------- use acc input ---------- */
      // double r, t, z;
      // r = rand_acc_r(eng);
      // t = rand_acc_t(eng);
      // z = rand_acc_z(eng);
      // Eigen::Vector3d acc(r * cos(t), r * sin(t), z);
      // obj_models[i].setInput(acc);

      /* ---------- use vel input ---------- */
      double vx, vy, vz, yd;
      vx = rand_vel(eng);
      vy = rand_vel(eng);
      vz = 0.0;
      yd = rand_yaw_dot(eng);

      obj_models[i].setInput(Eigen::Vector3d(vx, vy, vz));
      obj_models[i].setYawDot(yd);
    }
    time_change = time_now;
  }

  /* ---------- update obj state ---------- */
  double dt = (time_now - time_update).toSec();
  time_update = time_now;
  for (int i = 0; i < obj_num; ++i) {
    Eigen::Vector3d p_prev = obj_models[i].getPosition();
    obj_models[i].update(dt);
    // [Luan van - M4] cham vat can tinh: lui ve vi tri truoc, phan xa van toc qua phap tuyen
    Eigen::Vector3d push;
    if (hitStatic(obj_models[i].getPosition(), obj_models[i].getScale(), push)) {
      obj_models[i].setPosition(p_prev);
      Eigen::Vector3d v = obj_models[i].getVelocity();
      if (push.norm() < 0.5) v = -v;                              // khong ro huong: quay dau
      else if (v.dot(push) < 0.0) v -= 2.0 * v.dot(push) * push;  // phan xa guong
      obj_models[i].setVelocity(v(0), v(1), v(2));
    }
    visualizeObj(i);
  }

  /* ---------- collision ---------- */
  for (int i = 0; i < obj_num; ++i)
    for (int j = i + 1; j < obj_num; ++j) {
      bool collision = LinearObjModel::collide(obj_models[i], obj_models[j]);
      if (collision) {
        double yd1 = rand_yaw_dot(eng);
        double yd2 = rand_yaw_dot(eng);
        obj_models[i].setYawDot(yd1);
        obj_models[j].setYawDot(yd2);
      }
    }
}

void visualizeObj(int id) {
  Eigen::Vector3d pos, color, scale;
  pos = obj_models[id].getPosition();
  color = obj_models[id].getColor();
  scale = obj_models[id].getScale();
  double yaw = obj_models[id].getYaw();

  Eigen::Matrix3d rot;
  rot << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

  Eigen::Quaterniond qua;
  qua = rot;

  /* ---------- rviz ---------- */
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = id;

  mk.scale.x = scale(0), mk.scale.y = scale(1), mk.scale.z = scale(2);
  mk.color.a = 0.5, mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2);

  mk.pose.orientation.w = qua.w();
  mk.pose.orientation.x = qua.x();
  mk.pose.orientation.y = qua.y();
  mk.pose.orientation.z = qua.z();

  mk.pose.position.x = pos(0), mk.pose.position.y = pos(1), mk.pose.position.z = pos(2);

  obj_pub.publish(mk);

  /* ---------- pose ---------- */
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.seq = id;
  pose.pose.position.x = pos(0), pose.pose.position.y = pos(1), pose.pose.position.z = pos(2);
  pose.pose.orientation.w = 1.0;
  pose_pubs[id].publish(pose);
}
