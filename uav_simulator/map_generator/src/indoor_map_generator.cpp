// Bo sinh ban do Indoor — cong cu ngoai tuyen, khong can roscore.
//   rosrun map_generator indoor_map_generator I1 20260910 <duong_dan.pcd> [res]

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
static const double FLOOR_Z = 0.0;  // san that trung mat dat cua bo mo phong (z = 0)
// Mot lop diem phang. Cho x0==x1 de duoc mat phang doc theo YZ, v.v.
void addSlab(Cloud& c, double x0, double x1, double y0, double y1,
             double z0, double z1, double r = -1.0) {
  const double s = (r > 0.0) ? r : RES;
  for (double x = x0; x <= x1 + 1e-9; x += s)
    for (double y = y0; y <= y1 + 1e-9; y += s)
      for (double z = z0; z <= z1 + 1e-9; z += s)
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
  addSlab(c, -xh, xh, -yh, yh, FLOOR_Z, FLOOR_Z, 0.2);  // san, thua
  addSlab(c, -xh, xh, -yh, yh, sz, sz, 0.2);            // tran, thua
  addSlab(c, -xh, -xh, -yh, yh, FLOOR_Z, sz);           // tuong x-
  addSlab(c,  xh,  xh, -yh, yh, FLOOR_Z, sz);           // tuong x+
  addSlab(c, -xh, xh, -yh, -yh, FLOOR_Z, sz);           // tuong y-
  addSlab(c, -xh, xh,  yh,  yh, FLOOR_Z, sz);           // tuong y+
}

// ---------------- I1: phong don thua, 15x10x3, 8 cot ----------------
void buildI1(Cloud& c, unsigned seed) {
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
    addPillar(c, x, y, r, FLOOR_Z, SZ);
    ++placed;
  }
}

void buildI3(Cloud& c, unsigned /*seed*/) {
  const double SX = 20.0, SY = 20.0, SZ = 3.0;
  addRoomShell(c, SX, SY, SZ);

  //addSlab(c, -10.0, 9.0, -9.0, -9.0, 0.0, SZ);   // vach duoi nhanh ngang (ngoai goc)
  addSlab(c, -10.0, 7.0, -7.0, -7.0, FLOOR_Z, SZ);   // vach tren nhanh ngang (trong goc)
  //addSlab(c,   9.0, 9.0, -9.0, 10.0, 0.0, SZ);   // vach phai nhanh doc  (ngoai goc)
  addSlab(c,   7.0, 7.0, -7.0, 10.0, FLOOR_Z, SZ);   // vach trai nhanh doc  (trong goc)
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Dung: indoor_map_generator <I1..I5> <seed> <out.pcd> [res]\n";
    return 1;
  }
  std::string type = argv[1];
  unsigned    seed = (unsigned)std::strtoul(argv[2], NULL, 10);
  std::string out  = argv[3];
  if (argc >= 5) RES = std::atof(argv[4]);

  Cloud cloud;
  if      (type == "I1") buildI1(cloud, seed);
  else if (type == "I3") buildI3(cloud, seed);
  else { std::cerr << "Chua cai dat ban do: " << type << "\n"; return 1; }

  cloud.width    = cloud.points.size();
  cloud.height   = 1;
  cloud.is_dense = true;
  pcl::io::savePCDFileBinary(out, cloud);

  std::cout << type << "  seed=" << seed << "  res=" << RES
            << "  -> " << cloud.points.size() << " diem  -> " << out << "\n";
  return 0;
}