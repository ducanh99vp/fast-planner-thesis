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


// ---------------- I4: bon phong co cua, 25x15x3 ----------------
static const double WALL_T = 0.20;  // vach ngan day 0.2 m, sinh diem tren hai mat
static const double DOOR_H = 2.20;  // o cua cao 2.2 m, phia tren van la tuong

typedef std::vector<std::pair<double,double> > Doors;  // (tam, be rong) doc theo vach

// Vach ngan hai mat. along_y = true: vach x = pos chay theo y tu a0 den a1;
// along_y = false: vach y = pos chay theo x. Toa do doc vach goi la a.
void addPartition(Cloud& c, bool along_y, double pos, double a0, double a1,
                  double sz, const Doors& doors) {
  // doi (a, t, z) sang (x, y, z); t la toa do theo be day vach
  auto put = [&](double a, double t, double z) {
    if (along_y) c.points.push_back(pcl::PointXYZ(pos + t, a, z));
    else         c.points.push_back(pcl::PointXYZ(a, pos + t, z));
  };
  int na = (int)std::round((a1 - a0) / RES);
  int nz = (int)std::round(sz / RES);
  int nt = (int)std::round(WALL_T / RES);
  for (int i = 0; i <= na; ++i) {
    double a = a0 + i * RES;
    for (int k = 0; k <= nz; ++k) {
      double z = FLOOR_Z + k * RES;
      bool in_door = false;
      for (size_t d = 0; d < doors.size(); ++d)
        if (std::fabs(a - doors[d].first) < doors[d].second / 2.0 - 1e-6 && z < DOOR_H - 1e-6)
          in_door = true;
      if (in_door) continue;
      put(a, -WALL_T / 2.0, z);
      put(a,  WALL_T / 2.0, z);
    }
  }
  // bit kin o cua: hai ma cua va mat duoi da cua, khong de ho khe giua hai mat
  for (size_t d = 0; d < doors.size(); ++d) {
    double lo = doors[d].first - doors[d].second / 2.0;
    double hi = doors[d].first + doors[d].second / 2.0;
    int nw = (int)std::round((hi - lo) / RES);
    int nh = (int)std::round(DOOR_H / RES);
    for (int j = 0; j <= nt; ++j) {
      double t = -WALL_T / 2.0 + j * RES;
      for (int k = 0; k <= nh; ++k) {
        put(lo, t, FLOOR_Z + k * RES);
        put(hi, t, FLOOR_Z + k * RES);
      }
      for (int i = 0; i <= nw; ++i) put(lo + i * RES, t, DOOR_H);
    }
  }
}

// Hop do dac (ban, tu): mat tren va bon mat ben, khong day, khong dac ruot.
void addBox(Cloud& c, double cx, double cy, double sx, double sy, double sz) {
  double x0 = cx - sx / 2, x1 = cx + sx / 2, y0 = cy - sy / 2, y1 = cy + sy / 2;
  addSlab(c, x0, x1, y0, y1, FLOOR_Z + sz, FLOOR_Z + sz);  // mat tren
  addSlab(c, x0, x0, y0, y1, FLOOR_Z, FLOOR_Z + sz);        // mat x-
  addSlab(c, x1, x1, y0, y1, FLOOR_Z, FLOOR_Z + sz);        // mat x+
  addSlab(c, x0, x1, y0, y0, FLOOR_Z, FLOOR_Z + sz);        // mat y-
  addSlab(c, x0, x1, y1, y1, FLOOR_Z, FLOOR_Z + sz);        // mat y+
}

void buildI4(Cloud& c, unsigned /*seed*/) {
  const double SX = 25.0, SY = 15.0, SZ = 3.0;
  addRoomShell(c, SX, SY, SZ);

  // vach doc x = 0: cua A (y = -2.5, 1.4 m), cua B (y = +2.5, 1.2 m)
  addPartition(c, true,  0.0, -SY / 2, SY / 2, SZ, {{-2.5, 1.4}, {2.5, 1.2}});
  // vach ngang y = 0: cua C (x = -6, 1.2 m), cua D (x = +6, 0.9 m, hep nhat)
  addPartition(c, false, 0.0, -SX / 2, SX / 2, SZ, {{-6.0, 1.2}, {6.0, 0.9}});

  // Phan lon do dac cao 1.8-2.2 m de chan that su o do cao bay 1 m;
  // giu 2 ban thap 0.75 m de con tinh huong bay vuot ben tren.
  const double furn[][5] = {
    {-8.5, -4.0, 1.2, 0.8, 1.80}, {-4.5, -2.0, 0.8, 1.6, 2.00},   // phong Tay Nam
    {-8.0,  3.5, 1.6, 0.8, 0.75}, {-3.5,  5.0, 0.8, 0.8, 2.20},   // phong Tay Bac
    { 3.5, -4.5, 1.2, 0.8, 1.80}, { 7.5, -2.5, 0.8, 1.6, 2.00},   // phong Dong Nam
    { 6.0, -5.5, 1.0, 1.0, 2.20},
    { 3.0,  4.0, 1.6, 0.8, 0.75}, { 8.0,  2.5, 0.8, 1.2, 2.00},   // phong Dong Bac
  };

  for (const auto& f : furn) addBox(c, f[0], f[1], f[2], f[3], f[4]);
}

// ---------------- I5: ngo cut chu U, 20x15x3 ----------------
// Tuong chu U kin toi tran, mieng quay ve phia xuat phat (-8, 0), dich (8, 0)
// nam sau day chu U. Dung de ghi nhan cuc tieu dia phuong cua baseline.
void buildI5(Cloud& c, unsigned /*seed*/) {
  const double SX = 20.0, SY = 15.0, SZ = 3.0;
  addRoomShell(c, SX, SY, SZ);

  const double X_MIENG = -2.0, X_DAY = 2.0, Y_CANH = 4.0;   // tui sau 4 m, rong 8 m
  const Doors khong_cua;
  // day chu U: vach x = +2, keo dai qua be day canh de bit kin goc
  addPartition(c, true, X_DAY, -Y_CANH - WALL_T / 2, Y_CANH + WALL_T / 2, SZ, khong_cua);
  // hai canh chu U: vach y = +-4, tu mieng toi day
  addPartition(c, false,  Y_CANH, X_MIENG, X_DAY, SZ, khong_cua);
  addPartition(c, false, -Y_CANH, X_MIENG, X_DAY, SZ, khong_cua);
  // bit dau hai canh o mieng tui
  addSlab(c, X_MIENG, X_MIENG,  Y_CANH - WALL_T / 2,  Y_CANH + WALL_T / 2, FLOOR_Z, SZ);
  addSlab(c, X_MIENG, X_MIENG, -Y_CANH - WALL_T / 2, -Y_CANH + WALL_T / 2, FLOOR_Z, SZ);
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
  else if (type == "I4") buildI4(cloud, seed);
  else if (type == "I5") buildI5(cloud, seed);
  else { std::cerr << "Chua cai dat ban do: " << type << "\n"; return 1; }

  cloud.width    = cloud.points.size();
  cloud.height   = 1;
  cloud.is_dense = true;
  pcl::io::savePCDFileBinary(out, cloud);

  std::cout << type << "  seed=" << seed << "  res=" << RES
            << "  -> " << cloud.points.size() << " diem  -> " << out << "\n";
  return 0;
}