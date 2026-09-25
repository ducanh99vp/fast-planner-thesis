#ifndef _DYN_OBS_H_
#define _DYN_OBS_H_

#include <Eigen/Eigen>
#include <functional>
#include <memory>
#include <vector>

namespace fast_planner {

/* [Luan van - M6] Mo ta mot vat can DONG doc lap voi nguon du lieu.
 *
 * Bo lap ke hoach chi can ba cau hoi: da co du doan chua, tam hop o dau tai
 * thoi diem t, va hop to bao nhieu. Nguon tra loi co the la ObjPredictor
 * (ngoai suy tu quan sat) hoac ban tin B-spline cua UAV hang xom o M7a —
 * ham chi phi khong can biet su khac biet.
 *
 * `half` la ham chu khong phai gia tri, vi kich thuoc vat can den tu ban tin
 * Marker phat sau khi nut khoi dong, khong co san luc dang ky.
 */
struct DynObs {
  std::function<bool()>                          valid;
  std::function<Eigen::Vector3d(double)>         center;
  std::function<Eigen::Vector3d()>               half;
};

typedef std::shared_ptr<std::vector<DynObs>> DynObsList;

}  // namespace fast_planner
#endif
