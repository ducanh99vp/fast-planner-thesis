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



#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <plan_env/dyn_obs.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment {
private:
  /* data */
  ObjPrediction obj_prediction_;
  DynObsList dyn_obs_;  // [Luan van - M6] nguon vat can dong trung lap
  ObjScale obj_scale_;
  double resolution_inv_;
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
  EDTEnvironment(/* args */) {
  }
  ~EDTEnvironment() {
  }

  SDFMap::Ptr sdf_map_;

  void init();
  void setMap(SDFMap::Ptr map);
  void setObjPrediction(ObjPrediction prediction);
  /* [Luan van - M6] Dang ky danh sach vat can dong tu nguon bat ky. Khi da
     dang ky, getDynObsNum/getDynObsBox doc tu day thay vi tu ObjPrediction. */
  void setDynObsList(DynObsList list) { dyn_obs_ = list; }
  void setObjScale(ObjScale scale);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff,
                            double& value, Eigen::Vector3d& grad);
  void evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time,
                           double& dist, Eigen::Vector3d& grad);
  double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);
  /* [Luan van - M5] Truy van vat can DONG de bo toi uu hoa tu tinh chi phi f_d.
     `time` lay goc la ObjHistory::global_start_time_ — dung goc ma ObjHistory
     ghi moc thoi gian cua tung mau quan sat, KHONG phai thoi diem bat dau quy
     dao. Tra ve false khi vat can chua du 2 mau lich su nen chua co du doan. */
  int  getDynObsNum();
  bool getDynObsBox(int idx, const double& time, Eigen::Vector3d& center,
                    Eigen::Vector3d& half);

  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
    sdf_map_->getRegion(ori, size);
  }

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace fast_planner

#endif