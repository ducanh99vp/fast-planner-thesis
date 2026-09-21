#ifndef _BOX_DIST_H_
#define _BOX_DIST_H_

#include <Eigen/Eigen>
#include <cmath>

namespace fast_planner {

/* [Luan van - M5] Khoang cach CO DAU tu mot diem toi hop song song truc.
 *
 *   ben ngoai hop : d = || max(|e| - h, 0) ||  > 0
 *   ben trong hop : d = -(h_j* - |e_j*|)       < 0,  j* = truc co mat gan nhat
 *
 * voi e = pos - center, h = nua kich thuoc hop.
 *
 * Khac EDTEnvironment::distToBox: ham do tra ve 0 cho MOI diem ben trong hop,
 * nen gradient bang 0 va bo toi uu hoa mat luc day ra dung luc nguy hiem nhat.
 *
 * grad nhan dao ham cua d theo pos. Gia tri tra ve la d.
 */
inline double signedDistToBox(const Eigen::Vector3d& pos, const Eigen::Vector3d& center,
                              const Eigen::Vector3d& half, Eigen::Vector3d& grad) {
  Eigen::Vector3d e = pos - center;
  Eigen::Vector3d w;
  for (int i = 0; i < 3; ++i) w(i) = std::max(std::fabs(e(i)) - half(i), 0.0);

  double wn = w.norm();
  if (wn > 1e-12) { /* ben ngoai hop */
    for (int i = 0; i < 3; ++i) grad(i) = (e(i) >= 0.0 ? 1.0 : -1.0) * w(i) / wn;
    return wn;
  }

  /* ben trong hop: day ra theo mat gan nhat */
  int    js  = 0;
  double pen = half(0) - std::fabs(e(0));
  for (int i = 1; i < 3; ++i) {
    double pi = half(i) - std::fabs(e(i));
    if (pi < pen) { pen = pi; js = i; }
  }
  grad.setZero();
  grad(js) = (e(js) >= 0.0 ? 1.0 : -1.0);
  return -pen;
}

}  // namespace fast_planner
#endif
