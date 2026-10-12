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
#include <nav_msgs/Path.h>   // [CD2] nhan dich de bat dau chuyen dong
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include <plan_env/linear_obj_model.hpp>
#include <unordered_set>
using namespace std;

int obj_num;
double _xy_size, _h_size, _vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;

/* [CD2] Hinh dang, kich thuoc co dinh va chuyen dong THANG DEU THEO TRUC.
   Ban goc: hop ngau nhien, lo lung o do cao ngau nhien, bay huong ngau nhien.
   Ta can: tru 1.8 m x D 0.5 m va hop cao 3 m, DUNG TREN MAT DAT, di theo truc. */
bool   _axis_only;            // true = van toc luon nam tren mot truc
double _xy_x, _xy_y;          // nua be rong vung sinh vat can (x va y rieng)
double _cyl_d, _cyl_h;        // tru: duong kinh, chieu cao
double _box_x, _box_y, _box_h;// hop: canh x, canh y, chieu cao
vector<int> axis_of;          // 0 = truc x, 1 = truc y
vector<int> shape_of;         // 0 = hop, 1 = tru
uniform_real_distribution<double> rand_px, rand_py;

/* [CD2] Nay khoi VAT CAN TINH. Ban goc chi nay o bien ban do va khi hai vat can
   dong cham nhau; no di XUYEN qua cot va vanh tron. Ta nap dam may diem tinh vao
   mot tap o luoi roi kiem moi khung hinh. */
bool   _bounce_static;          // bat/tat
double _map_res;                // do phan giai o luoi khi bam ban do
double _unstick;                // giay; 0 = tat. Go ket o goc tuong
unordered_set<long> static_vox; // tap o luoi bi chiem boi vat can tinh
bool   map_ready  = false;
bool   objs_ready = false;   // [CD2] chua dat xong vat can thi chua duoc cap nhat
/* [CD2] Neo thoi diem xuat phat. Truoc day vat can chay ngay khi node khoi
   dong, con UAV chi bay sau khi ban do san sang + cho 4 s + cat canh. Khoang
   khoi dong do dao dong vai giay moi lan, nen CUNG hat giong ma pha giua UAV va
   vat can lech nhau (do duoc: khoang ho chenh 0.27 m). Giu vat can DUNG YEN cho
   toi khi nhan dich dau tien. */
bool   _start_on_goal = false;
bool   motion_started = false;
vector<Eigen::Vector3d> stuck_pos;
vector<double>          stuck_since;

inline long voxKey(double x, double y, double z) {
  const long ix = (long)floor(x / _map_res);
  const long iy = (long)floor(y / _map_res);
  const long iz = (long)floor(z / _map_res);
  return ((ix + 4096L) << 26) | ((iy + 4096L) << 13) | (iz + 4096L);
}

/* Hop vat can co cham o luoi tinh nao khong. Quet ca hop theo do phan giai. */
bool hitStatic(const Eigen::Vector3d& pos, const Eigen::Vector3d& scale) {
  if (!map_ready || static_vox.empty()) return false;
  const double hx = 0.5 * scale(0), hy = 0.5 * scale(1), hz = 0.5 * scale(2);
  for (double dx = -hx; dx <= hx + 1e-9; dx += _map_res)
    for (double dy = -hy; dy <= hy + 1e-9; dy += _map_res)
      for (double dz = -hz; dz <= hz + 1e-9; dz += _map_res)
        if (static_vox.count(voxKey(pos(0) + dx, pos(1) + dy, pos(2) + dz))) return true;
  return false;
}

void mapCallback(const sensor_msgs::PointCloud2& msg) {
  if (map_ready) return;                       // ban do tinh khong doi
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(msg, cloud);
  if (cloud.points.empty()) return;
  static_vox.reserve(cloud.points.size() * 2);
  for (const auto& pt : cloud.points)
    static_vox.insert(voxKey(pt.x, pt.y, pt.z));
  map_ready = true;
  ROS_INFO("[dynamic]: da nap ban do tinh, %lu o luoi", (unsigned long)static_vox.size());
}

ros::Publisher obj_pub;            // visualize marker
vector<ros::Publisher> pose_pubs;  // obj pose (from optitrack)
vector<LinearObjModel> obj_models;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_pos;
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

void goalCallback(const nav_msgs::Path& msg) {
  if (motion_started) return;
  motion_started = true;
  time_update = ros::Time::now();   // dt khoi dau bang 0, khong nhay vi tri
  time_change = ros::Time::now();
  ROS_INFO("[dynamic]: nhan dich -> vat can bat dau chuyen dong");
}


void updateCallback(const ros::TimerEvent& e);
void visualizeObj(int id);
void mapCallback(const sensor_msgs::PointCloud2& msg);  // [CD2]

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
  /* [CD2] tham so moi */
  node.param("obj_generator/axis_only", _axis_only, false);
  node.param("obj_generator/xy_x",   _xy_x,   8.0);
  node.param("obj_generator/xy_y",   _xy_y,   8.0);
  node.param("obj_generator/cyl_d",  _cyl_d,  0.5);
  node.param("obj_generator/cyl_h",  _cyl_h,  1.8);
  node.param("obj_generator/box_x",  _box_x,  0.6);
  node.param("obj_generator/box_y",  _box_y,  0.6);
  node.param("obj_generator/box_h",  _box_h,  3.0);
  node.param("obj_generator/bounce_static", _bounce_static, true);
  node.param("obj_generator/map_res",       _map_res,       0.1);
  node.param("obj_generator/unstick",       _unstick,       2.5);
  node.param("obj_generator/start_on_goal", _start_on_goal, false);
  /* [CD2] Gieo hat. Ban goc dung random_device nen MOI luot chay dat vat can
     o cho khac nhau -> khong the so cap hai phuong an tren cung kich ban.
     seed < 0 = giu hanh vi ngau nhien cu. */
  int _seed;
  node.param("obj_generator/seed", _seed, -1);
  if (_seed >= 0) {
    eng.seed((unsigned)_seed);
    ROS_INFO("[dynamic]: gieo hat = %d (tai lap duoc)", _seed);
  }

  ros::Subscriber map_sub =
      node.subscribe("/map_generator/global_cloud", 1, mapCallback);  // [CD2]
  ros::Subscriber goal_sub =
      node.subscribe("/waypoint_generator/waypoints", 1, goalCallback);  // [CD2]
  obj_pub = node.advertise<visualization_msgs::Marker>("/dynamic/obj", 10);
  for (int i = 0; i < obj_num; ++i) {
    ros::Publisher pose_pub =
        node.advertise<geometry_msgs::PoseStamped>("/dynamic/pose_" + to_string(i), 10);
    pose_pubs.push_back(pose_pub);
  }

  ros::Timer update_timer = node.createTimer(ros::Duration(1 / 30.0), updateCallback);
  cout << "[dynamic]: initialize with " + to_string(obj_num) << " moving obj." << endl;
  ros::Duration(1.0).sleep();

  /* [CD2] Doi ban do tinh toi TRUOC khi dat vat can. ros::Duration::sleep()
     KHONG xu ly callback, nen phai spinOnce. Khong co buoc nay thi vat can co
     the sinh ra NGAM TRONG mot cay cot: moi khung hinh hitStatic deu dung ->
     lui vi tri -> dung yen vinh vien. Da quan sat: 2/6 vat can bat dong. */
  if (_bounce_static) {
    ros::Rate wait_rate(50);
    int wait_n = 0;
    while (ros::ok() && !map_ready && wait_n < 500) {   // toi da 10 s
      ros::spinOnce();
      wait_rate.sleep();
      ++wait_n;
    }
    if (!map_ready)
      ROS_WARN("[dynamic]: het gio cho ban do tinh — bo qua loai tru vi tri dat");
  }

  rand_color = uniform_real_distribution<double>(0.0, 1.0);
  rand_pos = uniform_real_distribution<double>(-_xy_size, _xy_size);
  rand_px  = uniform_real_distribution<double>(-_xy_x, _xy_x);   // [CD2]
  rand_py  = uniform_real_distribution<double>(-_xy_y, _xy_y);   // [CD2]
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

    /* [CD2] Hinh dang xen ke: chan = HOP (cao 3 m, chan tran trong nha nen
       KHONG bay vuot len duoc), le = TRU (1.8 m, co nguoi di bo).
       Truc xen ke theo tung cap: 2 cai di NGANG, 2 cai di DOC. */
    const int shp = i % 2;        // 0 = hop, 1 = tru
    const int ax  = (i < obj_num / 2) ? 0 : 1;  // nua dau truc x, nua sau truc y
    shape_of.push_back(shp);
    axis_of.push_back(ax);

    Eigen::Vector3d scale = (shp == 0)
        ? Eigen::Vector3d(_box_x, _box_y, _box_h)
        : Eigen::Vector3d(_cyl_d, _cyl_d, _cyl_h);

    /* [CD2] Dat tren MAT DAT (tam o nua chieu cao), va KHONG duoc ngam trong
       vat can tinh — boc them 0.3 m de khong dat sat mat cot. */
    Eigen::Vector3d pos;
    Eigen::Vector3d probe = scale + Eigen::Vector3d(0.6, 0.6, 0.0);
    for (int tryk = 0; tryk < 200; ++tryk) {
      pos = Eigen::Vector3d(rand_px(eng), rand_py(eng), 0.5 * scale(2));
      if (!hitStatic(pos, probe)) break;
      if (tryk == 199)
        ROS_WARN("[dynamic]: vat can %d khong tim duoc cho dat trong, dat tam", i);
    }

    /* van toc nam tron tren mot truc, dau ngau nhien */
    const double sgn = (rand_vel(eng) >= 0.0) ? 1.0 : -1.0;
    Eigen::Vector3d vel = (ax == 0) ? Eigen::Vector3d(sgn * _vel, 0.0, 0.0)
                                    : Eigen::Vector3d(0.0, sgn * _vel, 0.0);

    /* mau: hop do, tru xanh — de phan biet tren RViz */
    Eigen::Vector3d color = (shp == 0) ? Eigen::Vector3d(0.85, 0.25, 0.20)
                                       : Eigen::Vector3d(0.20, 0.55, 0.85);

    /* yaw = 0 va khong tu xoay: hop phai song song truc thi hinh hoc va
       cham moi khop voi hop song song truc dung o cac buoc sau */
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    model.initialize(pos, vel, acc, 0.0, 0.0, color, scale);
    model.setLimits(Eigen::Vector3d(_xy_x, _xy_y, _h_size), Eigen::Vector2d(0.0, _vel),
                    Eigen::Vector2d(0, 0));
    obj_models.push_back(model);
    stuck_pos.push_back(pos);          // [CD2]
    stuck_since.push_back(0.0);        // [CD2]
  }

  time_update = ros::Time::now();
  time_change = ros::Time::now();
  objs_ready  = true;   // [CD2] tu day timer moi duoc phep chay

  /* ---------- start loop ---------- */
  ros::spin();

  return 0;
}

void updateCallback(const ros::TimerEvent& e) {
  /* [CD2] Vong cho ban do goi ros::spinOnce(), nen timer nay co the chay
     TRUOC khi obj_models duoc dat xong -> truy cap ngoai mang -> segfault.
     Da gap that: exit code -11. */
  if (!objs_ready) return;

  /* [CD2] chua co dich: chi ve vat can (dung yen), khong cap nhat vi tri */
  if (_start_on_goal && !motion_started) {
    for (int i = 0; i < obj_num; ++i) visualizeObj(i);
    time_update = ros::Time::now();
    time_change = ros::Time::now();
    return;
  }

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

      if (_axis_only) {
        /* [CD2] giu tren dung truc cua no, chi doi dau */
        const double sgn = (vx >= 0.0) ? 1.0 : -1.0;
        obj_models[i].setInput(axis_of[i] == 0 ? Eigen::Vector3d(sgn * _vel, 0.0, 0.0)
                                               : Eigen::Vector3d(0.0, sgn * _vel, 0.0));
        obj_models[i].setYawDot(0.0);
      } else {
        obj_models[i].setInput(Eigen::Vector3d(vx, vy, vz));
        obj_models[i].setYawDot(yd);
      }
    }
    time_change = time_now;
  }

  /* ---------- update obj state ---------- */
  double dt = (time_now - time_update).toSec();
  time_update = time_now;
  for (int i = 0; i < obj_num; ++i) {
    const Eigen::Vector3d p_prev = obj_models[i].getPosition();
    obj_models[i].update(dt);

    /* [CD2] Cham VAT CAN TINH: lui ve vi tri truoc roi dao chieu — giong het
       cach ban goc xu ly khi hai vat can dong cham nhau. O che do axis_only thi
       chi dao dau tren dung truc cua no, khong sinh thanh phan cheo. */
    if (_bounce_static && hitStatic(obj_models[i].getPosition(), obj_models[i].getScale())) {
      obj_models[i].setPosition(p_prev);
      Eigen::Vector3d v = obj_models[i].getVelocity();
      if (_axis_only) {
        const double sgn = (v(axis_of[i]) >= 0.0) ? -1.0 : 1.0;
        v = (axis_of[i] == 0) ? Eigen::Vector3d(sgn * _vel, 0.0, 0.0)
                              : Eigen::Vector3d(0.0, sgn * _vel, 0.0);
      } else {
        v = -v;
      }
      obj_models[i].setVelocity(v(0), v(1), v(2));
      obj_models[i].setInput(v);
    }
    visualizeObj(i);
  }

  /* [CD2] Go ket. Lui vi tri + dao chieu khong go duoc o GOC: dao xong lai cham
     ngay mat kia va dung yen mai. Neu khong nhuc nhich qua _unstick giay thi
     cuong buc dao chieu. */
  if (_unstick > 0.0) {
    const double tn = time_now.toSec();
    for (int i = 0; i < obj_num; ++i) {
      const Eigen::Vector3d p = obj_models[i].getPosition();
      if ((p - stuck_pos[i]).norm() > 0.05) {
        stuck_pos[i] = p;
        stuck_since[i] = tn;
      } else if (stuck_since[i] > 0.0 && tn - stuck_since[i] > _unstick) {
        Eigen::Vector3d v = obj_models[i].getVelocity();
        const double sgn = (v(axis_of[i]) >= 0.0) ? -1.0 : 1.0;
        v = (axis_of[i] == 0) ? Eigen::Vector3d(sgn * _vel, 0.0, 0.0)
                              : Eigen::Vector3d(0.0, sgn * _vel, 0.0);
        obj_models[i].setVelocity(v(0), v(1), v(2));
        obj_models[i].setInput(v);
        stuck_since[i] = tn;
      } else if (stuck_since[i] == 0.0) {
        stuck_since[i] = tn;
      }
    }
  }

  /* ---------- collision ---------- */
  for (int i = 0; i < obj_num; ++i)
    for (int j = i + 1; j < obj_num; ++j) {
      bool collision = LinearObjModel::collide(obj_models[i], obj_models[j]);
      if (collision && !_axis_only) {  // [CD2] axis_only thi giu yaw = 0
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
  /* [CD2] tru ve bang CYLINDER, hop ve bang CUBE */
  mk.type = (shape_of[id] == 1) ? visualization_msgs::Marker::CYLINDER
                                : visualization_msgs::Marker::CUBE;
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
