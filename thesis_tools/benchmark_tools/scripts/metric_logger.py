#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
metric_logger.py — Mốc M2, luận văn UAV tự hành vòng tránh vật cản

Node này làm ba việc trong MỘT lần chạy thử nghiệm:
  1. Chờ hệ thống sẵn sàng, rồi tự phát điểm đích (không cần bấm RViz)
  2. Thu thập toàn bộ chỉ số trong suốt chuyến bay
  3. Ghi một dòng CSV rồi tự tắt

Tự tắt là chủ đích: batch_runner.py dựa vào đó để biết lượt chạy đã xong.

Tham số ROS (đặt trong launch file):
  ~out_csv      đường dẫn file CSV để ghi thêm dòng
  ~map_name     tên bản đồ, ví dụ I1
  ~config       tên cấu hình thuật toán, ví dụ B0
  ~trial        số thứ tự lượt chạy
  ~goal_x  ~goal_y   toạ độ đích (z luôn bị FSM ép về 1.0)
  ~timeout      giây, quá thời gian này coi như thất bại
  ~goal_tol     mét, vào trong bán kính này coi như tới đích
  ~uav_radius   mét, gần vật cản hơn mức này coi như va chạm
"""

import os
import csv
import math
import threading

import numpy as np
import rospy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker   # [CD2] vat can dong
from plan_manage.msg import Bspline          # [CD2] quy dao duoc phat, de dem doi phia
import sensor_msgs.point_cloud2 as pc2

try:
    from scipy.spatial import cKDTree
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

try:
    from quadrotor_msgs.msg import PositionCommand
    HAVE_POSCMD = True
except ImportError:
    HAVE_POSCMD = False


FIELDS = [
    "map", "config", "trial",
    "success", "collided", "timed_out",
    "T_f", "L", "v_mean", "v_max", "v_cmd_max","v_cmd_axis_max", "S_J", "j_rms", "dacc_mean", "dacc_max", "d_min", "d_p5", "d_min_cmd", "z_min_flight",
    # [CD2] vat can dong: tom tat moi luot
    "n_enc", "n_hit", "d_min_dyn", "d_p5_dyn",
    "n_plan", "n_flip", "yaw_total",   # [CD2] do du / lat trai-phai
    "tilt_max", "tilt_p99", "n_tilt15",  # [CD2] mat thang bang: goc nghieng than (do)
    "N_replan", "N_fail",
    "t_fe_mean", "t_fe_max", "t_be_mean", "t_be_max", "t_be_p95",
    "start_x", "start_y", "goal_x", "goal_y",
    "n_odom", "stamp",
]


# [CD2] -------------------------------------------------------------------
# Khoang cach CO DAU tu mot diem toi be mat vat can.
#   > 0 : ben ngoai, la khoang ho that
#   < 0 : ben trong, la do xuyen sau  ->  VA CHAM
# Marker type: 1 = CUBE (hop song song truc), 3 = CYLINDER (truc thang dung).

ENC_CUBE, ENC_CYL = 1, 3


def sdf_box(p, c, half):
    """Hop song song truc."""
    e = np.abs(p - c) - half
    outside = float(np.linalg.norm(np.maximum(e, 0.0)))
    inside = float(min(np.max(e), 0.0))
    return outside + inside


def sdf_cyl(p, c, radius, half_h):
    """Tru thang dung: ban kinh trong mat phang xy, nua chieu cao theo z."""
    dxy = float(np.linalg.norm(p[:2] - c[:2])) - radius
    dz = abs(float(p[2] - c[2])) - half_h
    outside = math.sqrt(max(dxy, 0.0) ** 2 + max(dz, 0.0) ** 2)
    inside = min(max(dxy, dz), 0.0)
    return outside + inside


def sdf_obj(p, o):
    """o = dict(c, scale, type)."""
    c = o["c"]
    sc = o["scale"]
    if o["type"] == ENC_CYL:
        return sdf_cyl(p, c, 0.5 * sc[0], 0.5 * sc[2])
    return sdf_box(p, c, 0.5 * sc)


ENC_FIELDS = [
    "map", "config", "trial", "obj_id", "obj_type",
    "t_min", "d_min", "hit",
    "uav_x", "uav_y", "uav_z", "obj_x", "obj_y",
    "uav_speed", "obj_speed", "angle_deg",
]


class MetricLogger(object):

    def __init__(self):
        self.out_csv    = rospy.get_param("~out_csv", "/tmp/bench.csv")
        self.map_name   = rospy.get_param("~map_name", "unknown")
        self.config     = rospy.get_param("~config", "B0")
        self.trial      = int(rospy.get_param("~trial", 0))
        self.goal_x     = float(rospy.get_param("~goal_x", 5.0))
        self.goal_y     = float(rospy.get_param("~goal_y", 0.0))
        self.goal_z     = float(rospy.get_param("~goal_z", 1.0))   # [CD2] trung fsm/goal_z

        # [CD2] vat can dong -------------------------------------------------
        # enc_thresh: khoang cach bat dau tinh la MOT LAN CHAM TRAN.
        # enc_exit:   phai gian ra them chung nay moi coi la ket thuc, de mot
        #             lan tiep can khong bi dem thanh nhieu lan vi nhieu do.
        self.enc_csv     = rospy.get_param("~enc_csv", "")
        self.enc_thresh  = float(rospy.get_param("~enc_thresh", 3.0))
        self.enc_exit    = float(rospy.get_param("~enc_exit", 0.3))
        # tre: phai ra han enc_thresh + enc_hyst moi duoc mo lan moi,
        # neu khong thi dao dong quanh dung nguong se sinh lan gia
        self.enc_hyst    = float(rospy.get_param("~enc_hyst", 0.5))
        self.dyn_objs    = {}     # id -> dict(c, scale, type, v, t)
        self.enc_state   = {}     # id -> dict dang theo doi mot lan cham tran
        self.encounters  = []     # danh sach lan cham tran da ket thuc
        self.dyn_d_list  = []
        self.d_min_dyn   = float("inf")
        # [CD2] lat trai-phai. Moi quy dao moi: lay doan ~1-2.7 s phia truoc
        # (diem dieu khien 3..8), do do lech NGANG so voi vi tri hien tai.
        # Huong toi dich la truc x nen "ngang" la truc y. Bo qua khi lech qua
        # nho (|lat| < flip_min), roi dem so lan doi dau giua hai quy dao lien
        # tiep: do la so lan planner DOI Y lach trai <-> phai.
        self.flip_min    = float(rospy.get_param("~flip_min", 0.3))
        self.n_plan      = 0
        self.n_flip      = 0
        self.last_side   = 0
        self.yaw_prev    = None
        self.yaw_total   = 0.0
        self.tilts       = []      # [CD2] goc nghieng than moi mau odom (do)
        self.n_tilt15    = 0       # so LAN nghieng > 15 do (cach nhau >= 1 s)
        self.t_tilt15    = -1e9
        self.timeout    = float(rospy.get_param("~timeout", 120.0))
        self.goal_tol   = float(rospy.get_param("~goal_tol", 0.6))
        self.uav_radius = float(rospy.get_param("~uav_radius", 0.2))
        self.goal_delay = float(rospy.get_param("~goal_delay", 4.0))
        # [Luan van - M3] do cao cat canh (<= 0: tat); truoc khi cat canh xong khong xet va cham
        self.takeoff_h  = float(rospy.get_param("~takeoff_height", -1.0))
        self.v_cmd_axis_max = 0.0
        self.lock = threading.Lock()
        self.finished = False

        # --- trạng thái thu thập ---
        self.kdtree     = None
        self.cloud_n    = 0
        self.goal_sent  = False
        self.t_start    = None
        self.have_odom  = False

        self.prev_pos   = None
        self.path_len   = 0.0
        self.v_list     = []
        self.d_min      = float("inf")
        self.d_min_cmd  = float("inf")
        self.d_list     = []   # [Luan van - M3] moi khoang cach sau cat canh, de tinh d_p5
        self.airborne     = False   # [Luan van - M3] da cat canh xong chua
        self.z_min_flight = float("inf")
        self.n_odom     = 0
        self.start_pos  = None
        self.last_pos   = None

        self.prev_acc   = None
        self.prev_acc_t = None
        self.jerk_sq_int = 0.0
        self.prev_tid    = None
        self.joint_jumps = []
        self.v_cmd_max   = 0.0

        self.t_fe_list  = []
        self.t_be_list  = []
        self.n_replan   = 0
        self.n_fail     = 0

        self.collided   = False
        self.success    = False
        self.timed_out  = False

        # --- giao tiếp ROS ---
        self.goal_pub = rospy.Publisher(
            "/waypoint_generator/waypoints", Path, queue_size=1, latch=True)

        rospy.Subscriber("/map_generator/global_cloud", PointCloud2,
                         self.cb_cloud, queue_size=1)
        rospy.Subscriber("/dynamic/obj", Marker, self.cb_dynobj, queue_size=50)  # [CD2]
        rospy.Subscriber("/planning/bspline", Bspline, self.cb_bspline, queue_size=10)  # [CD2]
        rospy.Subscriber("/visual_slam/odom", Odometry, self.cb_odom, queue_size=50)
        rospy.Subscriber("/benchmark/timing", Float64MultiArray,
                         self.cb_timing, queue_size=50)
        if HAVE_POSCMD:
            rospy.Subscriber("/planning/pos_cmd", PositionCommand,
                             self.cb_cmd, queue_size=50)

        rospy.Timer(rospy.Duration(0.2), self.cb_tick)

        rospy.loginfo("[metric_logger] %s / %s / lượt %d — đích (%.2f, %.2f)",
                      self.map_name, self.config, self.trial,
                      self.goal_x, self.goal_y)
        if not HAVE_SCIPY:
            rospy.logwarn("[metric_logger] Thiếu scipy — d_min sẽ không đo được. "
                          "Cài bằng: pip3 install scipy")

    # ------------------------------------------------------------------
    # [CD2] ---------------------------------------------------------------
    def cb_dynobj(self, msg):
        """Moi message la MOT vat can. Luu vi tri, kich thuoc, loai, va uoc
        luong van toc bang sai phan giua hai lan nhan lien tiep."""
        with self.lock:
            if self.finished:
                return
            t = rospy.Time.now().to_sec()
            c = np.array([msg.pose.position.x, msg.pose.position.y,
                          msg.pose.position.z])
            sc = np.array([msg.scale.x, msg.scale.y, msg.scale.z])
            old = self.dyn_objs.get(msg.id)
            v = np.zeros(3)
            if old is not None and t - old["t"] > 1e-4:
                v = (c - old["c"]) / (t - old["t"])
                v = 0.6 * old["v"] + 0.4 * v      # lam tron nhe
            self.dyn_objs[msg.id] = {"c": c, "scale": sc, "type": msg.type,
                                     "v": v, "t": t}

    def cb_bspline(self, msg):
        """[CD2] Dem quy dao va so lan doi phia lach."""
        with self.lock:
            if self.finished or not self.goal_sent or len(msg.pos_pts) < 9:
                return
            self.n_plan += 1
            y0 = msg.pos_pts[1].y
            lat = sum(pt.y for pt in msg.pos_pts[3:9]) / 6.0 - y0
            if abs(lat) < self.flip_min:
                return
            side = 1 if lat > 0 else -1
            if self.last_side != 0 and side != self.last_side:
                self.n_flip += 1
            self.last_side = side

    def _update_encounters(self, p, v_uav, t):
        """Theo doi tung vat can; mot LAN CHAM TRAN la mot cuc tieu dia phuong
        cua khoang cach, nam duoi enc_thresh."""
        for oid, o in self.dyn_objs.items():
            d = sdf_obj(p, o)
            self.dyn_d_list.append(d)
            if d < self.d_min_dyn:
                self.d_min_dyn = d

            st = self.enc_state.get(oid)

            if st is None:
                # chua theo doi: chi mo khi lot vao trong nguong
                if d < self.enc_thresh:
                    self.enc_state[oid] = {
                        "d_min": d, "t_min": t, "closed": False,
                        "uav": p.copy(), "obj": o["c"].copy(),
                        "vu": v_uav.copy(), "vo": o["v"].copy()}
            elif st["closed"]:
                # DA dong: phai RA HAN khoi vung nguong moi duoc mo lan moi.
                # Khong co dieu kien nay thi mot lan tiep can duy nhat bi dem
                # thanh nhieu lan khi no dang roi xa (da gap: 1 lan -> 5 dong).
                if d >= self.enc_thresh + self.enc_hyst:
                    del self.enc_state[oid]
            else:
                if d < st["d_min"]:
                    st.update(d_min=d, t_min=t, uav=p.copy(),
                              obj=o["c"].copy(), vu=v_uav.copy(),
                              vo=o["v"].copy())
                elif d > st["d_min"] + self.enc_exit or d >= self.enc_thresh:
                    self._close_encounter(oid, st)

    def _close_encounter(self, oid, st):
        st["closed"] = True
        nu, no = np.linalg.norm(st["vu"][:2]), np.linalg.norm(st["vo"][:2])
        ang = float("nan")
        if nu > 1e-3 and no > 1e-3:
            cs = float(np.dot(st["vu"][:2], st["vo"][:2]) / (nu * no))
            ang = math.degrees(math.acos(max(-1.0, min(1.0, cs))))
        o = self.dyn_objs.get(oid, {})
        self.encounters.append({
            "map": self.map_name, "config": self.config, "trial": self.trial,
            "obj_id": oid, "obj_type": o.get("type", -1),
            "t_min": round(st["t_min"] - (self.t_start or 0.0), 3),
            "d_min": round(st["d_min"], 4),
            "hit": int(st["d_min"] < self.uav_radius),
            "uav_x": round(float(st["uav"][0]), 3),
            "uav_y": round(float(st["uav"][1]), 3),
            "uav_z": round(float(st["uav"][2]), 3),
            "obj_x": round(float(st["obj"][0]), 3),
            "obj_y": round(float(st["obj"][1]), 3),
            "uav_speed": round(float(nu), 3),
            "obj_speed": round(float(no), 3),
            "angle_deg": round(ang, 1) if ang == ang else "",
        })

    def cb_cloud(self, msg):
        if self.kdtree is not None or not HAVE_SCIPY:
            return
        pts = np.array(
            [p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"),
                                            skip_nans=True)],
            dtype=np.float32)
        if pts.shape[0] == 0:
            return
        self.kdtree = cKDTree(pts)
        self.cloud_n = pts.shape[0]
        rospy.loginfo("[metric_logger] Đã nạp bản đồ: %d điểm", self.cloud_n)

    # ------------------------------------------------------------------
    def cb_odom(self, msg):
        with self.lock:
            if self.finished:
                return
            p = np.array([msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.pose.pose.position.z])
            v = np.array([msg.twist.twist.linear.x,
                          msg.twist.twist.linear.y,
                          msg.twist.twist.linear.z])
            self.have_odom = True
            self.last_pos = p

            # [CD2] tong goc quay than UAV (do), tinh tu luc phat dich
            if self.goal_sent:
                q = msg.pose.pose.orientation
                yw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                if self.yaw_prev is not None:
                    dy = (yw - self.yaw_prev + math.pi) % (2 * math.pi) - math.pi
                    self.yaw_total += abs(dy)
                self.yaw_prev = yw
                # [CD2] goc giua truc z than va truc z the gioi
                tl = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
                self.tilts.append(tl)
                tn = msg.header.stamp.to_sec()
                if tl > 15.0:
                    if tn - self.t_tilt15 > 1.0:
                        self.n_tilt15 += 1
                    self.t_tilt15 = tn

            if not self.goal_sent:
                if self.start_pos is None:
                    self.start_pos = p.copy()
                return

            self.n_odom += 1
            if self.prev_pos is not None:
                self.path_len += float(np.linalg.norm(p - self.prev_pos))
            self.prev_pos = p
            self.v_list.append(float(np.linalg.norm(v)))

            # [Luan van - M3] san ban do o z = 0: UAV dang dau/cat canh luon "cham" san, nen chi
            # xet va cham va do cao sau khi da len toi takeoff_height - 0.1
            if not self.airborne and (self.takeoff_h <= 0 or p[2] >= self.takeoff_h - 0.1):
                self.airborne = True
            if self.airborne:
                self.z_min_flight = min(self.z_min_flight, float(p[2]))

            # khoảng cách tới vật cản gần nhất
            # [Luan van - M3] tinh tren MOI mau odom (truoc day moi 3 mau, bo sot ~2 cm o 1.5 m/s)
            if self.kdtree is not None and self.airborne:
                d, _ = self.kdtree.query(p.reshape(1, 3), k=1)
                d = float(d[0])
                self.d_list.append(d)

                if d < self.d_min:
                    self.d_min = d
                if d < self.uav_radius:
                    if not self.collided:
                        rospy.logwarn("[metric_logger] VA CHẠM: d = %.3f m", d)
                    self.collided = True

            # [CD2] theo doi cham tran voi vat can dong
            if self.airborne and self.dyn_objs:
                self._update_encounters(p, v, msg.header.stamp.to_sec()
                                        if msg.header.stamp.to_sec() > 0
                                        else rospy.Time.now().to_sec())

            # tới đích chưa
            goal = np.array([self.goal_x, self.goal_y, self.goal_z])   # [CD2]
            if np.linalg.norm(p - goal) < self.goal_tol:
                self.success = True
                self._finish("tới đích")

    # ------------------------------------------------------------------
    def cb_cmd(self, msg):
        """Tính jerk bằng sai phân số trên gia tốc lệnh."""
        with self.lock:
            if self.finished or not self.goal_sent:
                return
            t = msg.header.stamp.to_sec()
            a = np.array([msg.acceleration.x,
                          msg.acceleration.y,
                          msg.acceleration.z])
            tid = msg.trajectory_id
            vc = float(np.linalg.norm([msg.velocity.x,
                                       msg.velocity.y,
                                       msg.velocity.z]))
            va = max(abs(msg.velocity.x), abs(msg.velocity.y), abs(msg.velocity.z))
            if va > self.v_cmd_axis_max:
                self.v_cmd_axis_max = va                           
            if vc > self.v_cmd_max:
                self.v_cmd_max = vc
            # khoang cach tu VI TRI LENH toi ban do that — cung KD-tree voi d_min
            # [Luan van - M3] bo qua luc dang cat canh (lenh con sat san z = 0)
            if self.kdtree is not None and self.airborne:
                pc = np.array([msg.position.x, msg.position.y, msg.position.z])
                dc, _ = self.kdtree.query(pc.reshape(1, 3), k=1)
                dc = float(dc[0])
                if dc < self.d_min_cmd:
                    self.d_min_cmd = dc
            if self.prev_acc is not None:
                dt = t - self.prev_acc_t
                if 1e-4 < dt < 0.2:
                    if tid == self.prev_tid:
                        # cùng một quỹ đạo: jerk thật, cộng vào S_J
                        j = (a - self.prev_acc) / dt
                        self.jerk_sq_int += float(np.dot(j, j)) * dt
                    else:
                        # mối nối giữa hai quỹ đạo: ghi riêng, không cộng vào S_J
                        self.joint_jumps.append(
                            float(np.linalg.norm(a - self.prev_acc)))
            self.prev_acc   = a
            self.prev_acc_t = t
            self.prev_tid   = tid

    # ------------------------------------------------------------------
    def cb_timing(self, msg):
        """Bản tin từ planner_manager: [t_fe_ms, t_be_ms, thành_công]."""
        with self.lock:
            if self.finished or len(msg.data) < 3:
                return
            t_fe, t_be, ok = msg.data[0], msg.data[1], msg.data[2]
            if ok > 0.5:
                self.t_fe_list.append(t_fe)
                self.t_be_list.append(t_be)
                self.n_replan += 1
            else:
                self.n_fail += 1

    # ------------------------------------------------------------------
    def cb_tick(self, _evt):
        with self.lock:
            if self.finished:
                return
            now = rospy.Time.now().to_sec()

            if not self.goal_sent:
                if self.have_odom and self.kdtree is not None:
                    if not hasattr(self, "_ready_at"):
                        self._ready_at = now
                    elif now - self._ready_at >= self.goal_delay:
                        self._send_goal()
                return

            if now - self.t_start > self.timeout:
                self.timed_out = True
                self._finish("hết thời gian")

    # ------------------------------------------------------------------
    def _send_goal(self):
        path = Path()
        path.header.frame_id = "world"
        path.header.stamp = rospy.Time.now()
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = self.goal_x
        ps.pose.position.y = self.goal_y
        ps.pose.position.z = self.goal_z   # [CD2] FSM dung fsm/goal_z
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
        self.goal_pub.publish(path)

        self.goal_sent = True
        self.t_start = rospy.Time.now().to_sec()
        self.prev_pos = self.last_pos.copy() if self.last_pos is not None else None
        rospy.loginfo("[metric_logger] Đã phát đích, bắt đầu đo.")

    # ------------------------------------------------------------------
    def _finish(self, reason):
        if self.finished:
            return
        self.finished = True
        T_f = rospy.Time.now().to_sec() - self.t_start if self.t_start else 0.0
        v = np.array(self.v_list) if self.v_list else np.array([0.0])
        fe = np.array(self.t_fe_list) if self.t_fe_list else np.array([0.0])
        be = np.array(self.t_be_list) if self.t_be_list else np.array([0.0])
        dacc  = np.array(self.joint_jumps) if self.joint_jumps else np.array([0.0])
        j_rms = math.sqrt(self.jerk_sq_int / T_f) if T_f > 1e-6 else 0.0

        # [CD2] dong moi lan cham tran con dang mo
        for oid, st in self.enc_state.items():
            if not st["closed"]:
                self._close_encounter(oid, st)
        dd = np.array(self.dyn_d_list) if self.dyn_d_list else np.array([float("nan")])
        n_hit = sum(e["hit"] for e in self.encounters)

        row = {
            "map": self.map_name,
            "config": self.config,
            "trial": self.trial,
            "success": int(self.success and not self.collided),
            "collided": int(self.collided),
            "timed_out": int(self.timed_out),
            "T_f": round(T_f, 3),
            "L": round(self.path_len, 3),
            "v_mean": round(float(v.mean()), 3),
            "v_max": round(float(v.max()), 3),
            "v_cmd_max": round(self.v_cmd_max, 3),
            "v_cmd_axis_max": round(self.v_cmd_axis_max, 3),
            "S_J": round(self.jerk_sq_int, 3),
            "j_rms": round(j_rms, 3),
            "dacc_mean": round(float(dacc.mean()), 3),
            "dacc_max": round(float(dacc.max()), 3),
            "d_min": round(self.d_min, 4) if math.isfinite(self.d_min) else "",
            # [Luan van - M3] phan vi 5%: 95% thoi gian bay UAV cach vat can it nhat d_p5
            "d_p5": round(float(np.percentile(self.d_list, 5)), 4) if self.d_list else "",
            "d_min_cmd": round(self.d_min_cmd, 4) if math.isfinite(self.d_min_cmd) else "",
            "z_min_flight": round(self.z_min_flight, 3) if math.isfinite(self.z_min_flight) else "",
            "N_replan": self.n_replan,
            "N_fail": self.n_fail,
            "t_fe_mean": round(float(fe.mean()), 3),
            "t_fe_max": round(float(fe.max()), 3),
            "t_be_mean": round(float(be.mean()), 3),
            "t_be_max": round(float(be.max()), 3),
            "t_be_p95": round(float(np.percentile(be, 95)), 3),
            "start_x": round(float(self.start_pos[0]), 3) if self.start_pos is not None else "",
            "start_y": round(float(self.start_pos[1]), 3) if self.start_pos is not None else "",
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            # [CD2]
            "n_enc": len(self.encounters),
            "n_hit": n_hit,
            "d_min_dyn": round(float(self.d_min_dyn), 4)
                         if self.d_min_dyn < float("inf") else "",
            "d_p5_dyn": round(float(np.nanpercentile(dd, 5)), 4)
                        if self.dyn_d_list else "",
            "n_plan": self.n_plan,
            "n_flip": self.n_flip,
            "yaw_total": round(math.degrees(self.yaw_total), 1),
            "tilt_max": round(max(self.tilts), 1) if self.tilts else float("nan"),
            "tilt_p99": round(float(np.percentile(self.tilts, 99)), 1) if self.tilts else float("nan"),
            "n_tilt15": self.n_tilt15,
            "n_odom": self.n_odom,
            "stamp": rospy.Time.now().to_sec(),
        }

        d = os.path.dirname(self.out_csv)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        new_file = not os.path.isfile(self.out_csv)
        # [Luan van - M3] chan lech cot am tham: tieu de CSV cu phai trung FIELDS
        if not new_file:
            with open(self.out_csv, newline="") as fcsv:
                old = fcsv.readline().strip().split(",")
            if old != FIELDS:
                rospy.logfatal("[metric_logger] Tieu de CSV cu khong khop FIELDS - hay xoa %s", self.out_csv)
                rospy.signal_shutdown("lech luoc do CSV")
                return
        with open(self.out_csv, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            if new_file:
                w.writeheader()
            w.writerow(row)

        # [CD2] ghi tung lan cham tran ra CSV rieng — day moi la du lieu chinh
        if self.enc_csv and self.encounters:
            de = os.path.dirname(self.enc_csv)
            if de and not os.path.isdir(de):
                os.makedirs(de)
            new_e = not os.path.isfile(self.enc_csv)
            with open(self.enc_csv, "a", newline="") as fe:
                we = csv.DictWriter(fe, fieldnames=ENC_FIELDS)
                if new_e:
                    we.writeheader()
                for e in self.encounters:
                    we.writerow(e)

        rospy.loginfo("[metric_logger] Cham tran voi vat can dong: %d lan, "
                      "%d lan va cham, d_min_dyn=%s",
                      len(self.encounters), n_hit, str(row["d_min_dyn"]))

        rospy.loginfo("[metric_logger] Kết thúc (%s) | thành công=%d | "
                      "T_f=%.1fs | L=%.1fm | d_min=%s | replan=%d",
                      reason, row["success"], row["T_f"], row["L"],
                      str(row["d_min"]), row["N_replan"])
        rospy.Timer(rospy.Duration(1.0),
                    lambda e: rospy.signal_shutdown("xong lượt chạy"),
                    oneshot=True)


if __name__ == "__main__":
    rospy.init_node("metric_logger")
    MetricLogger()
    rospy.spin()
