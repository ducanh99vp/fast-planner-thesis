// Bo sinh ban do Indoor — cong cu ngoai tuyen, khong can roscore.
//   rosrun map_generator indoor_map_generator M1 20260910 <duong_dan.pcd> [res]

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

static double RES = 0.05;   // khoang cach giua hai diem lan can

// Mot lop diem phang. Cho x0==x1 de duoc mat phang doc theo YZ, v.v.
void addSlab(Cloud& c, double x0, double x1, double y0, double y1,
             double z0, double z1) {
  for (double x = x0; x <= x1 + 1e-9; x += RES)
    for (double y = y0; y <= y1 + 1e-9; y += RES)
      for (double z = z0; z <= z1 + 1e-9; z += RES)
        c.points.push_back(pcl::PointXYZ(x, y, z));
}

// Chi sinh MAT BEN cua tru, khong dac ruot.
// Camera do sau khong nhin thay ben trong, va d_min chi phu thuoc be mat.
void addPillar(Cloud& c, double cx, double cy, double r,
               double z0, double z1) {
  double dth = RES / r;                      // buoc goc ~ RES tren chu vi
  for (double th = 0.0; th < 2.0 * M_PI; th += dth)
    for (double z = z0; z <= z1 + 1e-9; z += RES)
      c.points.push_back(pcl::PointXYZ(cx + r * std::cos(th),
                                       cy + r * std::sin(th), z));
}

// Vo phong: san, tran, bon tuong bao.
void addRoomShell(Cloud& c, double sx, double sy, double sz) {
  double xh = sx / 2.0, yh = sy / 2.0;
  addSlab(c, -xh, xh, -yh, yh, sz,  sz);     // tran
  addSlab(c, -xh, -xh, -yh, yh, 0.0, sz);    // tuong x-
  addSlab(c,  xh,  xh, -yh, yh, 0.0, sz);    // tuong x+
  addSlab(c, -xh, xh, -yh, -yh, 0.0, sz);    // tuong y-
  addSlab(c, -xh, xh,  yh,  yh, 0.0, sz);    // tuong y+
}

// ---------------- M1: phong don thua, 15x10x3, 8 cot ----------------
void buildM1(Cloud& c, unsigned seed) {
  const double SX = 15.0, SY = 10.0, SZ = 3.0;
  addRoomShell(c, SX, SY, SZ);

  const double start_x = -6.0, goal_x = 6.0, clear_r = 1.5;
  std::mt19937 eng(seed);
  std::uniform_real_distribution<double> rx(-SX / 2 + 1.0, SX / 2 - 1.0);
  std::uniform_real_distribution<double> ry(-SY / 2 + 1.0, SY / 2 - 1.0);
  std::uniform_real_distribution<double> rr(0.20, 0.35);

  std::vector<std::pair<double,double> > centers;
  const double min_sep = 1.5;
  int placed = 0, guard = 0;
  while (placed < 8 && ++guard < 10000) {
    double x = rx(eng), y = ry(eng), r = rr(eng);
    if (std::hypot(x - start_x, y) < clear_r) continue;
    if (std::hypot(x - goal_x,  y) < clear_r) continue;
    bool too_close = false;
    for (size_t i = 0; i < centers.size(); ++i)
      if (std::hypot(x - centers[i].first, y - centers[i].second) < min_sep)
        { too_close = true; break; }
    if (too_close) continue;
    centers.push_back(std::make_pair(x, y));
    addPillar(c, x, y, r, 0.0, SZ);
    ++placed;
  }
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Dung: indoor_map_generator <M1..M5> <seed> <out.pcd> [res]\n";
    return 1;
  }
  std::string type = argv[1];
  unsigned    seed = (unsigned)std::strtoul(argv[2], NULL, 10);
  std::string out  = argv[3];
  if (argc >= 5) RES = std::atof(argv[4]);

  Cloud cloud;
  if      (type == "M1") buildM1(cloud, seed);
  else { std::cerr << "Chua cai dat ban do: " << type << "\n"; return 1; }

  cloud.width    = cloud.points.size();
  cloud.height   = 1;
  cloud.is_dense = true;
  pcl::io::savePCDFileBinary(out, cloud);

  std::cout << type << "  seed=" << seed << "  res=" << RES
            << "  -> " << cloud.points.size() << " diem  -> " << out << "\n";
  return 0;
}