// [Luan van - M5] Kiem thu gradient cua signedDistToBox va cua chi phi f_d
// bang sai phan so trung tam. Bien dich khong can ROS:
//   cd ~/fast_planner_ws/src/Fast-Planner
//   g++ -O2 -Ifast_planner/plan_env/include -I/usr/include/eigen3 \
//       thesis_tools/tests/test_box_grad.cpp -o /tmp/test_box_grad
#include <plan_env/box_dist.h>

#include <cmath>
#include <cstdio>
#include <random>

using fast_planner::signedDistToBox;

/* chi phi mot diem dieu khien voi mot vat can: f = (d - d0)^2 khi d < d0 */
static double costOne(const Eigen::Vector3d& p, const Eigen::Vector3d& c,
                      const Eigen::Vector3d& h, double d0, Eigen::Vector3d& grad) {
  Eigen::Vector3d gd;
  double          d = signedDistToBox(p, c, h, gd);
  if (d >= d0) { grad.setZero(); return 0.0; }
  grad = 2.0 * (d - d0) * gd;
  return (d - d0) * (d - d0);
}

int main() {
  const double d0 = 0.7, eps = 1e-6, tol = 1e-5;
  Eigen::Vector3d c(2.0, 1.0, 1.0), h(0.3, 0.4, 1.5);

  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> U(-1.0, 1.0);

  int n_tong = 0, n_bo = 0, n_hong = 0;
  double sai_max = 0.0;

  for (int k = 0; k < 200000; ++k) {
    Eigen::Vector3d p(c(0) + 2.0 * U(rng), c(1) + 2.0 * U(rng), c(2) + 2.5 * U(rng));

    /* bo cac diem KHONG kha vi:
       - sat mat hop theo mot truc ma cac truc khac cung sat  (canh / goc)
       - ben trong hop ma hai truc hoa nhau lam mat gan nhat   */
    Eigen::Vector3d e = p - c;
    double m[3];
    for (int i = 0; i < 3; ++i) m[i] = std::fabs(e(i)) - h(i);   /* <0: trong theo truc i */
    int  n_sat = 0;
    for (int i = 0; i < 3; ++i) if (std::fabs(m[i]) < 1e-3) ++n_sat;
    bool trong = (m[0] < 0 && m[1] < 0 && m[2] < 0);
    bool hoa   = false;
    if (trong) {
      double s[3] = { -m[0], -m[1], -m[2] };                     /* do sau toi tung mat */
      for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
          if (std::fabs(s[i] - s[j]) < 1e-3) hoa = true;
    }
    if (n_sat > 0 || hoa) { ++n_bo; continue; }

    Eigen::Vector3d g;
    costOne(p, c, h, d0, g);

    Eigen::Vector3d gn;
    for (int i = 0; i < 3; ++i) {
      Eigen::Vector3d pp = p, pm = p, tmp;
      pp(i) += eps; pm(i) -= eps;
      gn(i) = (costOne(pp, c, h, d0, tmp) - costOne(pm, c, h, d0, tmp)) / (2.0 * eps);
    }

    ++n_tong;
    double sai = (g - gn).norm() / std::max(1.0, gn.norm());
    if (sai > sai_max) sai_max = sai;
    if (sai > tol) {
      if (++n_hong <= 5)
        printf("  HONG  p=(%.4f,%.4f,%.4f)  giai tich=(%+.5f,%+.5f,%+.5f)  so=(%+.5f,%+.5f,%+.5f)\n",
               p(0), p(1), p(2), g(0), g(1), g(2), gn(0), gn(1), gn(2));
    }
  }

  printf("Diem da kiem : %d  (bo %d diem tren canh/goc/hoa, khong kha vi)\n", n_tong, n_bo);
  printf("Sai lech lon nhat: %.3e   nguong: %.1e\n", sai_max, tol);

  /* lien tuc khi di qua mat hop: d va gradient phai khop hai ben */
  Eigen::Vector3d gA, gB;
  Eigen::Vector3d a(c(0), c(1) + h(1) - 1e-7, c(2));
  Eigen::Vector3d b(c(0), c(1) + h(1) + 1e-7, c(2));
  double dA = signedDistToBox(a, c, h, gA), dB = signedDistToBox(b, c, h, gB);
  printf("Qua mat hop  : d %.2e -> %.2e (lech %.1e), gradient lech %.1e\n",
         dA, dB, std::fabs(dA - dB), (gA - gB).norm());

  /* dau cua d */
  Eigen::Vector3d gt;
  printf("Dau cua d    : tam hop %+.3f (phai am), xa hop %+.3f (phai duong)\n",
         signedDistToBox(c, c, h, gt), signedDistToBox(c + Eigen::Vector3d(3, 0, 0), c, h, gt));

  if (n_hong == 0 && (gA - gB).norm() < 1e-6) { printf("\nKET QUA: DAT\n"); return 0; }
  printf("\nKET QUA: HONG (%d diem sai)\n", n_hong);
  return 1;
}
