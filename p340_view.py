# coding=utf-8
"""ultraArm P340 四轴监控面板（新增模块，不改写六轴主视图）。

由 joint_monitor.py 的「机械臂型号」选择卡切换显示。P340 是 3~4 轴步进码垛臂，
读数接口为 get_angles_info() / get_coords_info()，关节限位非对称、运动学为平面连杆，
因此独立实现一套面板，与六轴球腕 3D 视图完全解耦。

    joint_monitor.py --model p340            # 命令行直接进 P340
    界面左列选择卡 → P340                    # 运行时切换

参数来源：
  https://www.elephantrobotics.com/ultraarm-p340/cn
  https://www.elephantrobotics.com/ultraarm-p340-spcn/
  https://docs.elephantrobotics.com/docs/ultraArm/
具体数值集中在 robot_profiles.py，本模块只负责呈现与采集。
"""

from __future__ import annotations

import math
import queue
import threading
import time

import tkinter as tk
from tkinter import ttk

import robot_profiles as profiles

# 与主界面一致的浅色主题；独立定义，避免反向 import joint_monitor
BG = "#F4F6F9"
CARD = "#FFFFFF"
TEXT = "#1F2937"
MUTED = "#6B7280"
GRID = "#E5E7EB"
TRACK = "#E9EDF2"
FLOOR = "#DDE3EB"
JOINT_COLORS = ("#E5484D", "#F59E0B", "#10B981", "#8B5CF6")
FONT_UI = ("Microsoft YaHei UI", 10)
FONT_UI_B = ("Microsoft YaHei UI", 10, "bold")
FONT_NUM = ("Consolas", 15, "bold")
FONT_SMALL = ("Microsoft YaHei UI", 9)
FONT_TINY = ("Microsoft YaHei UI", 8)

HISTORY = 120
L1, L2 = 110.0, 95.0        # 平面姿态示意连杆长度（可视化，非真实 mm）
BASE_H = 26.0

# 与主界面「动作」下拉一致，四个预设全部提供 P340 四轴版
DEMO_PRESETS = ("自由摆动", "逐关节演示", "抓取循环", "归零静止")


def demo_angles(t, preset=DEMO_PRESETS[0], n=3):
    """四种演示预设的 P340 版本，3 / 4 轴通用，角度均落在官方限位内。

    返回 (angles, active_joint)；active_joint 供「逐关节演示」高亮。
    """
    n = max(3, min(4, int(n)))
    if preset == "归零静止":
        return [0.0] * n, -1
    if preset == "逐关节演示":
        slot = int(t / 2.5)
        idx = slot % n
        phase = (t - slot * 2.5) / 2.5
        base = [0.0, 35.0, 35.0, 0.0][:n]
        amp = [80.0, 40.0, 30.0, 90.0][:n]
        base[idx] += amp[idx] * math.sin(2.0 * math.pi * phase)
        return [round(x, 2) for x in base], idx
    if preset == "抓取循环":
        w = 2.0 * math.pi * (t % 8.0) / 8.0
        vals = [
            60.0 * math.sin(w),
            35.0 + 30.0 * math.cos(w),
            35.0 + 25.0 * math.sin(w + 0.6),
            70.0 * math.sin(2.0 * w),
        ]
        return [round(x, 2) for x in vals[:n]], -1
    # 自由摆动（默认）
    vals = [
        80.0 * math.sin(0.40 * t),
        35.0 + 40.0 * math.sin(0.55 * t + 0.7),
        35.0 + 30.0 * math.sin(0.70 * t + 1.4),
        90.0 * math.sin(0.60 * t + 2.1),
    ]
    return [round(x, 2) for x in vals[:n]], -1


# ---- P340 3D 姿态数学（独立实现，不改动六轴主程序） ----
POSE_GRID = 120.0        # 地面网格半径（可视化单位）
L3 = 26.0                # 腕 / 工具段长度
RING_R = 12.0            # 关节轴环半径
POSE_TOP, POSE_BOTTOM, POSE_SIDE = 34, 26, 10


def m3mul(a, b):
    return [[a[i][0]*b[0][j] + a[i][1]*b[1][j] + a[i][2]*b[2][j]
             for j in range(3)] for i in range(3)]


def m3col(m, j):
    return [m[0][j], m[1][j], m[2][j]]


def rot_z(deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rot_y(deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def rot_x(deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def vadd(p, v, k=1.0):
    return [p[0] + v[0]*k, p[1] + v[1]*k, p[2] + v[2]*k]


def vcross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def vnorm(v):
    n = math.sqrt(sum(x*x for x in v))
    return [x/n for x in v] if n > 1e-9 else [0.0, 0.0, 1.0]


def ortho_basis(axis):
    u = vnorm(axis)
    ref = [0.0, 0.0, 1.0] if abs(u[2]) < 0.9 else [1.0, 0.0, 0.0]
    v = vnorm(vcross(u, ref))
    return v, vnorm(vcross(u, v))


def project(point, az, el, scale, ox, oy):
    """正交投影，返回 (screen_x, screen_y, depth)，depth 越大越远。"""
    ca, sa = math.cos(az), math.sin(az)
    x1 = ca*point[0] - sa*point[1]
    y1 = sa*point[0] + ca*point[1]
    z1 = point[2]
    ce, se = math.cos(el), math.sin(el)
    sy = z1*ce - y1*se
    depth = y1*ce + z1*se
    return ox + x1*scale, oy - sy*scale, depth


def build_arm_3d(angles, n=3):
    """J1 绕世界 Z；J2/J3 肩肘共面俯仰（局部 Y）；J4 绕连杆轴 Z 自转。

    连杆始终沿局部 Z；n=3 时忽略 J4 角度（选配轴）。
    """
    a = [float(x or 0.0) for x in list(angles)[:4]]
    a += [0.0] * (4 - len(a))
    segments, joints, frames = [], [], []
    p = [0.0, 0.0, 0.0]
    top = [0.0, 0.0, BASE_H]
    segments.append((p, top, 0))
    p = top
    joints.append((p, [0.0, 0.0, 1.0], 0))       # J1 轴：世界 Z
    R = rot_z(a[0])
    frames.append(R)
    d0 = m3col(R, 0)
    pointer = [d0[0]*30.0, d0[1]*30.0, 6.0]
    joints.append((p, m3col(R, 1), 1))           # J2 轴：局部 Y
    R = m3mul(R, rot_y(a[1]))
    frames.append(R)
    p1 = vadd(p, m3col(R, 2), L1)
    segments.append((p, p1, 1)); p = p1
    joints.append((p, m3col(R, 1), 2))           # J3 轴：局部 Y
    R = m3mul(R, rot_y(-a[2]))
    frames.append(R)
    p2 = vadd(p, m3col(R, 2), L2)
    segments.append((p, p2, 2)); p = p2
    joints.append((p, m3col(R, 2), 3))           # J4 轴：连杆轴 Z
    R = m3mul(R, rot_z(a[3] if n == 4 else 0.0))
    frames.append(R)
    tip = vadd(p, m3col(R, 2), L3)
    segments.append((p, tip, 3)); p = tip
    ex, ez = m3col(R, 0), m3col(R, 2)
    cross = (vadd(p, ex, -12.0), vadd(p, ex, 12.0))
    fingers = [(vadd(p, ex, sign*7.0),
                vadd(vadd(p, ex, sign*7.0), ez, 12.0)) for sign in (-1.0, 1.0)]
    return {"segments": segments, "joints": joints, "frames": frames,
            "tip": p, "frame": R, "base_pointer": pointer, "base_h": BASE_H,
            "cross": cross, "fingers": fingers}


def pose_points_3d(arm):
    """3D 视图中所有需要保证可见的点（含地面网格四角）。"""
    pts = []
    for p0, p1, _ in arm["segments"]:
        pts.extend((p0, p1))
    for center, _axis, _idx in arm["joints"]:
        pts.append(center)
    pts.extend(arm["cross"])
    for root, tip in arm["fingers"]:
        pts.extend((root, tip))
    pts.extend((arm["base_pointer"], arm["tip"]))
    for x in (-POSE_GRID, POSE_GRID):
        for y in (-POSE_GRID, POSE_GRID):
            pts.append([x, y, 0.0])
    return pts


def pose3d_selftest():
    """纯软件自检：旋转矩阵正交、坐标有限、多尺寸投影不越界。"""
    for ang in ([0.0]*4, [40.0, 35.0, 35.0, 60.0], [-90.0, 60.0, 60.0, -120.0]):
        arm = build_arm_3d(ang, 4)
        Rm = arm["frame"]
        for i in range(3):
            assert abs(sum(Rm[i][k]**2 for k in range(3)) - 1.0) < 1e-9
        for i in range(3):
            for j in range(i+1, 3):
                assert abs(sum(Rm[i][k]*Rm[j][k] for k in range(3))) < 1e-9
        for p0, p1, _ in arm["segments"]:
            for v in (p0, p1):
                assert all(math.isfinite(x) for x in v), "3D 点非有限值"
        pts = pose_points_3d(arm)
        for W, H in ((330, 360), (240, 180), (700, 520)):
            for az, el in ((0.0, 0.4), (0.9, 1.1), (2.4, 0.0)):
                raw = [project(v, az, el, 1.0, 0.0, 0.0) for v in pts]
                xs = [r[0] for r in raw]; ys = [r[1] for r in raw]
                spanx = max(1.0, max(xs)-min(xs)); spany = max(1.0, max(ys)-min(ys))
                sc = min(max(30.0, W-2*POSE_SIDE)/spanx,
                         max(30.0, H-POSE_TOP-POSE_BOTTOM)/spany) * 0.92
                sc = max(0.01, min(2.5, sc))
                ox = W/2 - (min(xs)+max(xs))/2*sc
                oy = POSE_TOP + max(30.0, H-POSE_TOP-POSE_BOTTOM)/2 - (min(ys)+max(ys))/2*sc
                for v in pts:
                    sx, sy, _d = project(v, az, el, sc, ox, oy)
                    assert -2.0 <= sx <= W+2 and -2.0 <= sy <= H+2, \
                        "画布 {}x{} 3D 点越界".format(W, H)
    return True


class _P340Source:
    """P340 真机 / 演示数据源。真机走官方 G-code 文本协议读数接口。"""

    def __init__(self, profile, port, baud, demo, axis_mode_getter, preset_getter=None):
        self.profile = profile
        self.port = port
        self.baud = baud
        self.demo = demo
        self.axis_mode_getter = axis_mode_getter
        self.preset_getter = preset_getter or (lambda: DEMO_PRESETS[0])
        self.robot = None
        self._n = 0
        self._coords = None

    def open(self):
        if self.demo:
            return
        if not self.port:
            raise ValueError("未选择串口，请先点击「刷新」扫描端口")
        self.robot = profiles.create_robot(self.profile, self.port, self.baud)

    def read(self):
        t0 = time.perf_counter()
        n = self.axis_mode_getter()
        if self.demo:
            angles, active = demo_angles(time.monotonic(), self.preset_getter(), n)
            ms = (time.perf_counter() - t0) * 1000.0
            return angles, None, ms, active

        self._n += 1
        angles = profiles.read_angles(self.robot, self.profile)
        if self._n % 5 == 1:
            coords = profiles.read_coords(self.robot, self.profile)
            if coords:
                self._coords = coords
        ms = (time.perf_counter() - t0) * 1000.0
        if not angles or len(angles) < n:      # 无响应时保留上一帧
            return None, self._coords, ms, -1
        return [float(x) for x in angles[:n]], self._coords, ms, -1

    def close(self):
        try:
            if self.robot is not None:
                self.robot._serial_port.close()
        except Exception:
            pass
        self.robot = None


class P340Panel(tk.Frame):
    """四轴监控面板。对外接口：start / stop / close / set_axis_mode。"""

    def __init__(self, parent, on_status=None):
        super().__init__(parent, bg=BG)
        self.profile = profiles.get_profile("p340")
        self.on_status = on_status or (lambda _text: None)
        self.axis_var = tk.IntVar(value=self.profile.dof)   # 3 / 4 轴
        self._axis_n = self.profile.dof                    # 采集线程读取，避免跨线程访问 tk 变量
        self._preset = DEMO_PRESETS[0]                     # 采集线程读取当前演示预设
        self._resize_job = None
        self.pose_mode = tk.StringVar(value="3d")          # 3D 姿态 / 2D 侧视
        self.view_az = math.radians(-35.0)
        self.view_el = math.radians(24.0)
        self.view_zoom = 1.0
        self._drag = None
        self.auto_fit = 1.0
        self._fit_snap = False
        self.q = queue.Queue()
        self.thread = None
        self.stop_evt = threading.Event()
        self.running = False
        self.demo = True
        self.port = ""
        self.baud = self.profile.baud
        self.interval = 0.2
        self.frames = 0
        self.history = []
        self.last_angles = [0.0] * self.profile.max_joints
        self.last_coords = None
        self.active_joint = -1
        self._build()
        self.after(60, self._draw_all)
        self.after(40, self._poll)

    # ---------------------------------------------------------------- 布局
    def _build(self):
        head = tk.Frame(self, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        head.pack(fill="x", padx=10, pady=(10, 6))
        tk.Label(head, text="ultraArm P340 · 四轴码垛步进机械臂监控", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(side="left", padx=10, pady=8)
        self.spec_label = tk.Label(head, text=profiles.summary(self.profile), bg=CARD,
                                   fg=MUTED, font=FONT_SMALL)
        self.spec_label.pack(side="left", padx=4)
        tk.Label(head, text="自由度", bg=CARD, fg=MUTED, font=FONT_SMALL).pack(
            side="left", padx=(12, 2))
        for n in self.profile.dof_options:
            tk.Radiobutton(head, text="{} 轴".format(n), value=n, variable=self.axis_var,
                           indicatoron=False, bg="#EDF1F6", selectcolor="#DBEAFE",
                           fg=TEXT, font=FONT_SMALL, relief="flat", padx=8, pady=2,
                           command=self.set_axis_mode).pack(side="left", padx=2, pady=8)

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        body.grid_columnconfigure(0, weight=2, minsize=250)
        body.grid_columnconfigure(1, weight=5, minsize=330)
        body.grid_columnconfigure(2, weight=3, minsize=250)
        body.grid_rowconfigure(0, weight=1)

        # 左：关节角度 / 限位条
        left = tk.Frame(body, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        tk.Label(left, text="关节角度（0 居中，左右按各自限位）", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=12, pady=(12, 2))
        tk.Label(left, text="3 轴为常规配置；第 4 轴选配", bg=CARD, fg=MUTED,
                 font=FONT_SMALL).pack(anchor="w", padx=12, pady=(0, 6))
        self.value_labels = []
        self.name_labels = []
        self.bar_canvases = []
        self.row_frames = []
        for i, name in enumerate(self.profile.joint_names):
            row = tk.Frame(left, bg=CARD)
            row.pack(fill="x", padx=12, pady=5)
            self.row_frames.append(row)
            h = tk.Frame(row, bg=CARD)
            h.pack(fill="x")
            lo, hi = self._limit_at(i)
            name_lab = tk.Label(h, text="{}  ({:+.0f}~{:+.0f})".format(name, lo, hi),
                                bg=CARD, fg=JOINT_COLORS[i], font=FONT_UI_B)
            name_lab.pack(side="left")
            self.name_labels.append(name_lab)
            v = tk.Label(h, text="--.--", bg=CARD, fg=TEXT, font=FONT_NUM, width=8, anchor="e")
            v.pack(side="right")
            self.value_labels.append(v)
            c = tk.Canvas(row, width=210, height=18, bg=TRACK, highlightthickness=0)
            c.pack(fill="x", pady=(4, 0))
            self.bar_canvases.append(c)
        tk.Label(left, text="供电 12V 5A · 串口 115200\n读数 get_angles_info() / get_coords_info()",
                 bg=CARD, fg=MUTED, font=FONT_SMALL, justify="left").pack(
            anchor="w", padx=12, pady=(14, 10))

        # 中：3D 姿态（可切 2D 侧视）
        mid = tk.Frame(body, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        mid.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        tk.Label(mid, text="姿态视图（J1 基座 / J2 大臂 / J3 小臂 / J4 选配）",
                 bg=CARD, fg=TEXT, font=FONT_UI_B).pack(anchor="w", padx=12, pady=(12, 2))
        mode_row = tk.Frame(mid, bg=CARD)
        mode_row.pack(fill="x", padx=12, pady=(0, 2))
        for txt, val in (("3D 姿态", "3d"), ("2D 侧视", "2d")):
            tk.Radiobutton(mode_row, text=txt, value=val, variable=self.pose_mode,
                           indicatoron=False, command=self._draw_pose, bg="#EDF1F6",
                           selectcolor="#DBEAFE", fg=TEXT, font=FONT_SMALL,
                           relief="flat", padx=8, pady=1).pack(side="left", padx=2)
        tk.Button(mode_row, text="复位视角", command=self._pose_reset, font=FONT_SMALL,
                  relief="flat", bg="#EDF1F6", fg=TEXT, padx=8).pack(side="right")
        self.pose = tk.Canvas(mid, width=330, height=360, bg=CARD, highlightthickness=0)
        self.pose.pack(fill="both", expand=True, padx=12, pady=(0, 2))
        self.pose.bind("<ButtonPress-1>", self._pose_press)
        self.pose.bind("<B1-Motion>", self._pose_drag)
        self.pose.bind("<MouseWheel>", self._pose_wheel)
        self.pose.bind("<Double-Button-1>", lambda e: self._pose_reset())
        tk.Label(mid, text="拖动旋转 / 滚轮缩放 / 双击复位；几何示意，非精确 DH 解算",
                 bg=CARD, fg=MUTED, font=FONT_SMALL).pack(anchor="w", padx=12, pady=(0, 10))

        # 右：趋势 + 末端坐标
        right = tk.Frame(body, bg=BG)
        right.grid(row=0, column=2, sticky="nsew")
        f1 = tk.Frame(right, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        f1.pack(fill="both", expand=True, pady=(0, 8))
        tk.Label(f1, text="角度趋势（最近 {} 帧）".format(HISTORY), bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=12, pady=(8, 2))
        self.trend = tk.Canvas(f1, width=230, height=190, bg=CARD, highlightthickness=0)
        self.trend.pack(fill="both", expand=True, padx=12, pady=(0, 2))
        legend = tk.Frame(f1, bg=CARD)
        legend.pack(pady=(0, 8))
        self.legend_labels = []
        for i in range(self.profile.max_joints):
            lab = tk.Label(legend, text="J{}".format(i + 1), bg=CARD, fg=JOINT_COLORS[i],
                           font=FONT_SMALL)
            lab.pack(side="left", padx=6)
            self.legend_labels.append(lab)

        f2 = tk.Frame(right, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        f2.pack(fill="x")
        tk.Label(f2, text="末端坐标（真机，每 5 帧刷新）", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=12, pady=(8, 2))
        grid = tk.Frame(f2, bg=CARD)
        grid.pack(fill="x", padx=12, pady=(0, 10))
        self.coord_labels = []
        for i, nm in enumerate(self.profile.coord_names):
            tk.Label(grid, text=nm, bg=CARD, fg=MUTED, font=FONT_SMALL, width=2).grid(
                row=i // 2, column=(i % 2) * 2, padx=(0, 2), pady=2, sticky="w")
            lab = tk.Label(grid, text="--", bg="#F3F5F8", fg=TEXT,
                           font=("Consolas", 10), width=9, anchor="e")
            lab.grid(row=i // 2, column=(i % 2) * 2 + 1, padx=(0, 8), pady=2, sticky="ew")
            self.coord_labels.append(lab)
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, _evt=None):
        """尺寸变化后防抖重绘，保证 0 线始终落在画布中线。"""
        if self._resize_job is not None:
            try:
                self.after_cancel(self._resize_job)
            except Exception:
                pass
        self._resize_job = self.after(80, self._draw_all)

    # ---------------------------------------------------------------- 数据
    def _limits(self):
        n = self.axis_var.get()
        return (self.profile.joint_limits_alt if n == 4 and self.profile.joint_limits_alt
                else self.profile.joint_limits)

    def _limit_at(self, i):
        """安全取第 i 个关节限位：3 轴模式下 J4 行仍需要标签数值。"""
        lim = self._limits()
        if i < len(lim):
            return lim[i]
        alt = self.profile.joint_limits_alt
        return alt[i] if alt and i < len(alt) else (-180.0, 180.0)

    def set_preset(self, preset):
        """主界面「动作」下拉切换时同步生效，采集过程中也可实时切换。"""
        if preset not in DEMO_PRESETS:
            return
        self._preset = preset
        self.on_status("P340 演示动作切换为：" + preset)

    def set_axis_mode(self):
        """3 轴 / 4 轴切换：官方标注自由度 3-4，第 4 轴为选配。"""
        self._axis_n = self.axis_var.get()
        self.history = []
        for i in range(self.profile.max_joints):
            show = i < self.axis_var.get()
            if show:
                self.row_frames[i].pack(fill="x", padx=12, pady=5)
                self.legend_labels[i].pack(side="left", padx=6)
                lo, hi = self._limit_at(i)
                self.name_labels[i].configure(
                    text="{}  ({:+.0f}~{:+.0f})".format(
                        self.profile.joint_names[i], lo, hi))
            else:
                self.row_frames[i].pack_forget()
                self.legend_labels[i].pack_forget()
        self._draw_all()
        self.on_status("P340 切换为 {} 轴模式".format(self.axis_var.get()))

    def start(self, port="", baud=None, demo=True, interval_ms=200,
              preset=DEMO_PRESETS[0]):
        """由主程序「开始采集」按钮委托调用。"""
        if self.running:
            self.stop()
        self.port = port
        self.baud = self.profile.baud if baud is None else baud
        self.demo = demo
        self._preset = preset if preset in DEMO_PRESETS else DEMO_PRESETS[0]
        self.interval = max(0.05, interval_ms / 1000.0)
        self.history = []
        self.frames = 0
        self.stop_evt = threading.Event()
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        self.on_status("P340 {}· {} 采集中...".format(
            "演示模式" if demo else "真机", self._preset))

    def _reader(self):
        src = _P340Source(self.profile, self.port, self.baud, self.demo,
                          lambda: self._axis_n, lambda: self._preset)
        try:
            src.open()
            self.q.put(("ready", None))
            while not self.stop_evt.is_set():
                self.q.put(("data", src.read()))
                self.stop_evt.wait(self.interval)
        except Exception as exc:
            self.q.put(("error", "{}: {}".format(type(exc).__name__, exc)))
        finally:
            src.close()
            self.q.put(("closed", None))

    def stop(self):
        self.stop_evt.set()
        self.running = False
        self.on_status("P340 已停止采集")

    def close(self):
        self.stop()
        try:
            if self.thread is not None:
                self.thread.join(timeout=1.5)
        except Exception:
            pass

    def _poll(self):
        try:
            if not self.winfo_exists():
                return
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "error":
                    self.on_status("P340 错误: " + str(payload))
                    self.running = False
                elif kind == "data":
                    self._update(*payload)
                elif kind == "closed":
                    self.running = False
        except queue.Empty:
            pass
        except tk.TclError:
            return
        self.after(40, self._poll)

    def _update(self, angles, coords, elapsed_ms, active=-1):
        self.frames += 1
        self.active_joint = active
        if angles is None:
            self.on_status("P340 警告: 机械臂无响应（检查 12V 供电 / 串口 / 115200 波特率）")
            return
        n = self.axis_var.get()
        vals = list(angles[:n])
        self.history.append(vals)
        if len(self.history) > HISTORY:
            self.history.pop(0)
        for i in range(self.profile.max_joints):
            v = vals[i] if i < len(vals) else None
            self.last_angles[i] = 0.0 if v is None else v
            self.value_labels[i].configure(
                text="{:+.2f}".format(v) if v is not None else "--.--",
                fg=(JOINT_COLORS[i] if i == active else TEXT))
            lo, hi = self._limit_at(i)
            self._draw_bar(self.bar_canvases[i], v, lo, hi, JOINT_COLORS[i])
        self._draw_pose(self.last_angles)
        self._draw_trend()
        if coords:
            for i, lab in enumerate(self.coord_labels):
                try:
                    lab.configure(text="{:.1f}".format(float(coords[i])))
                except (IndexError, TypeError, ValueError):
                    lab.configure(text="--")
        limits = self._limits()
        over = profiles.over_limit(self.profile, vals, limits)
        warn = " | 超限位 J{}".format(",".join(str(j) for j in over)) if over else ""
        mode = "演示" if self.demo else "真机 " + (self.port or "-")
        self.on_status("P340 | {} | {} | 帧 {} | 读取 {:.1f} ms{}{}".format(
            mode, self._preset if self.demo else "真机数据", self.frames, elapsed_ms,
            " | 演示 J{}".format(active + 1) if active >= 0 else "", warn))

    # ---------------------------------------------------------------- 绘图
    @staticmethod
    def _draw_bar(c, value, lo, hi, color):
        """0 线固定在轨道中点；左半区映射 [lo,0]，右半区映射 [0,hi]。

        与六轴主界面的 ±量程条保持同一视觉基准：正值向右、负值向左，
        非对称限位只影响左右各自的刻度密度，不再把 0 线挤到一边。
        """
        c.delete("all")
        w = c.winfo_width()
        if w < 20:
            w = c.winfo_reqwidth()
        w = max(120, w)
        h = c.winfo_height()
        if h < 8:
            h = c.winfo_reqheight()
        h = max(12, h)
        c.create_rectangle(0, 0, w, h, fill=TRACK, outline="")
        pad = 3
        mid = w / 2.0
        left_w = max(1.0, mid - pad)
        right_w = max(1.0, w - pad - mid)
        neg_span = max(1e-6, -lo) if lo < 0 else 1e-6
        pos_span = max(1e-6, hi) if hi > 0 else 1e-6

        def x_of(v):
            if v >= 0:
                return mid + min(v, pos_span) / pos_span * right_w
            return mid - min(-v, neg_span) / neg_span * left_w

        c.create_line(mid, 0, mid, h, fill="#8B95A5", width=1.5)   # 0 基准线
        c.create_line(pad, 1, pad, h - 1, fill="#C7CDD6")
        c.create_line(w - pad, 1, w - pad, h - 1, fill="#C7CDD6")
        if value is None:
            return
        over = value < lo or value > hi
        x = x_of(value)
        c.create_rectangle(min(mid, x), 2, max(mid, x), h - 2,
                           fill="#DC2626" if over else color, outline="")
        c.create_line(x, 0, x, h, fill=TEXT, width=2)
        if over:
            c.create_text(w - pad - 4, h / 2.0, anchor="e", fill="#FFFFFF",
                          font=FONT_TINY, text="超限")

    def _draw_trend(self):
        c = self.trend
        c.delete("all")
        w = max(120, c.winfo_width())
        h = max(80, c.winfo_height())
        pl, pr, pt, pb = 30, 8, 10, 18
        plot_w = w - pl - pr
        plot_h = h - pt - pb
        n = self.axis_var.get()
        limits = self._limits()
        for i in range(5):
            y = pt + plot_h * i / 4.0
            c.create_line(pl, y, w - pr, y, fill=GRID)
        c.create_text(pl - 4, pt, anchor="e", fill=MUTED, font=FONT_TINY, text="上限")
        c.create_text(pl - 4, pt + plot_h, anchor="e", fill=MUTED, font=FONT_TINY, text="下限")
        if len(self.history) < 2:
            return
        for j in range(n):
            lo, hi = limits[j]
            span = max(1.0, hi - lo)
            pts = []
            for i, row in enumerate(self.history):
                if j >= len(row):
                    continue
                x = pl + (i / max(1, HISTORY - 1)) * plot_w
                frac = max(0.0, min(1.0, (row[j] - lo) / span))
                y = pt + (1.0 - frac) * plot_h
                pts.extend((x, y))
            if len(pts) >= 4:
                c.create_line(*pts, fill=JOINT_COLORS[j], width=1.6)
        c.create_text(pl, h - 8, anchor="w", fill=MUTED, font=FONT_TINY, text="-{} 帧".format(HISTORY))
        c.create_text(w - pr, h - 8, anchor="e", fill=MUTED, font=FONT_TINY, text="现在")

    def _draw_pose(self, angles=None):
        """按当前模式分派：默认 3D 姿态，可切 2D 侧视。"""
        if angles is None:
            angles = self.last_angles
        if self.pose_mode.get() == "2d":
            return self._draw_pose_2d(angles)
        return self._draw_pose_3d(angles)

    # ---- 3D 视角交互
    def _pose_press(self, evt):
        self._drag = (evt.x, evt.y, self.view_az, self.view_el)

    def _pose_drag(self, evt):
        if not self._drag:
            return
        x0, y0, az0, el0 = self._drag
        self.view_az = az0 + (evt.x - x0) * 0.010
        self.view_el = max(-0.25, min(1.30, el0 + (evt.y - y0) * 0.007))
        self._draw_pose(self.last_angles)

    def _pose_wheel(self, evt):
        self.view_zoom = max(0.55, min(2.0, self.view_zoom * (1.10 if evt.delta > 0 else 0.90)))
        self._draw_pose(self.last_angles)

    def _pose_reset(self):
        self.view_az = math.radians(-35.0)
        self.view_el = math.radians(24.0)
        self.view_zoom = 1.0
        self.auto_fit = 1.0
        self._fit_snap = True
        self._draw_pose(self.last_angles)
        self._fit_snap = False

    def _fit_pose_3d(self, arm, W, H):
        """按所有 3D 点包围盒求缩放与居中，保证完整可见。"""
        pts = pose_points_3d(arm)
        raw = [project(v, self.view_az, self.view_el, 1.0, 0.0, 0.0) for v in pts]
        xs = [r[0] for r in raw]; ys = [r[1] for r in raw]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
        spanx = max(1.0, maxx-minx); spany = max(1.0, maxy-miny)
        avail_w = max(30.0, W - 2*POSE_SIDE)
        avail_h = max(30.0, H - POSE_TOP - POSE_BOTTOM)
        target = min(avail_w/spanx, avail_h/spany) * 0.92
        target = max(0.01, min(2.50, target))
        if self._fit_snap:
            self.auto_fit = target
        else:
            self.auto_fit += (target - self.auto_fit) * 0.18
        sc = self.auto_fit * self.view_zoom
        ox = W/2 - (minx+maxx)/2*sc
        oy = POSE_TOP + avail_h/2 - (miny+maxy)/2*sc
        return sc, ox, oy

    def _draw_pose_3d(self, angles):
        c = self.pose
        c.delete("all")
        W = c.winfo_width(); H = c.winfo_height()
        if W < 40: W = 330
        if H < 40: H = 360
        n = self.axis_var.get()
        arm = build_arm_3d(angles, n)
        sc, ox, oy = self._fit_pose_3d(arm, W, H)
        az, el = self.view_az, self.view_el

        def P(v):
            return project(v, az, el, sc, ox, oy)

        items = []
        def add_line(p0, p1, color, width):
            s0, s1 = P(p0), P(p1)
            d = (s0[2]+s1[2])/2.0
            items.append((d, len(items),
                          lambda s0=s0, s1=s1, color=color, width=width:
                          c.create_line(s0[0], s0[1], s1[0], s1[1], fill=color,
                                        width=width, capstyle=tk.ROUND)))
        def add_poly(points, color, width):
            sp = [P(v) for v in points]
            d = sum(v[2] for v in sp)/len(sp)
            flat = [x for v in sp for x in (v[0], v[1])]
            items.append((d, len(items),
                          lambda flat=flat, color=color, width=width:
                          c.create_line(*flat, fill=color, width=width, capstyle=tk.ROUND)))
        def add_dot(p3, r, fill, outline, width=1):
            s = P(p3)
            items.append((s[2], len(items),
                          lambda s=s, r=r, fill=fill, outline=outline, width=width:
                          c.create_oval(s[0]-r, s[1]-r, s[0]+r, s[1]+r,
                                        fill=fill, outline=outline, width=width)))

        g = POSE_GRID
        for k in range(-2, 3):
            t = k * g * 46.0 / 96.0
            add_line([t, -g, 0.0], [t, g, 0.0], FLOOR, 1)
            add_line([-g, t, 0.0], [g, t, 0.0], FLOOR, 1)
        add_dot([0, 0, 0], 24, "#CBD5E1", "#94A3B8", 1)
        add_line([0, 0, 0], [0, 0, arm["base_h"]], "#94A3B8", 10)
        add_line([0, 0, 6.0], arm["base_pointer"], JOINT_COLORS[0], 3)
        for p0, p1, idx in arm["segments"]:
            col = "#94A3B8" if idx == 0 else JOINT_COLORS[idx]
            add_line(p0, p1, "#CBD5E1", 11)
            add_line(p0, p1, col, 6)
        if n == 4:
            add_line(arm["cross"][0], arm["cross"][1], JOINT_COLORS[3], 4)
            for root, tip in arm["fingers"]:
                add_line(root, tip, JOINT_COLORS[3], 4)
        for center, axis, idx in arm["joints"]:
            if idx >= n:
                continue
            v, w = ortho_basis(axis)
            r = RING_R + (2.0 if idx == self.active_joint else 0.0)
            ring = [vadd(vadd(center, v, r*math.cos(t*math.pi/12.0)),
                         w, r*math.sin(t*math.pi/12.0)) for t in range(24)]
            add_poly(ring + [ring[0]], JOINT_COLORS[idx],
                     3.5 if idx == self.active_joint else 2.0)
        for center, _axis, idx in arm["joints"]:
            if idx >= n:
                continue
            fill = JOINT_COLORS[idx] if idx == self.active_joint else CARD
            add_dot(center, 6 if idx == self.active_joint else 5, fill, TEXT, 2)
        add_dot(arm["tip"], 4, "#EF4444", CARD, 1)
        for _d, _seq, fn in sorted(items, key=lambda it: (-it[0], it[1])):
            fn()
        for center, _axis, idx in arm["joints"]:
            if idx >= n:
                continue
            sp = P(center)
            c.create_text(sp[0]+12, sp[1]-11, text="J{}".format(idx+1),
                          fill=JOINT_COLORS[idx], font=("Consolas", 9, "bold"), anchor="w")
        cell = W / float(n)
        for i in range(n):
            on = (i == self.active_joint)
            c.create_rectangle(i*cell+3, 6, (i+1)*cell-3, 24,
                               fill=JOINT_COLORS[i] if on else "#EEF2F7", outline="")
            txt = ("J{} {:+.0f}".format(i+1, angles[i] or 0.0)
                   if cell >= 62 else "J{}".format(i+1))
            c.create_text(i*cell+cell/2, 15, text=txt,
                          fill="#FFFFFF" if on else JOINT_COLORS[i], font=FONT_TINY)
        state = ("当前演示 J{}".format(self.active_joint+1)
                 if self.active_joint >= 0 else "四轴联动")
        hint = ("拖动旋转 | 滚轮缩放 | 双击复位    " + state) if W >= 470 else state
        c.create_text(10, H-12, anchor="w", fill=MUTED, font=FONT_SMALL, text=hint)
        if W >= 430:
            c.create_text(W-10, H-12, anchor="e", fill=MUTED, font=FONT_TINY,
                          text="3D 几何示意 / 非精确 DH 解算")

    def _draw_pose_2d(self, angles):
        c = self.pose
        c.delete("all")
        W = max(200, c.winfo_width())
        H = max(200, c.winfo_height())
        sc = min((W - 60) / (2 * (L1 + L2) + 40), (H - 70) / (L1 + L2 + 60))
        sc = max(0.35, min(1.3, sc))
        ox, oy = W / 2.0, H - 45.0
        ground = oy
        for k in range(-4, 5):
            x = ox + k * 40 * sc
            c.create_line(x, ground, x - 12 * sc, ground + 18 * sc, fill=FLOOR)
        c.create_line(ox - 150 * sc, ground, ox + 150 * sc, ground, fill="#94A3B8", width=2)
        # 基座 + J1 回转指针（俯视投影成一枚随 J1 摆动的箭头）
        shoulder_y = oy - BASE_H * sc
        c.create_rectangle(ox - 26 * sc, oy - 8 * sc, ox + 26 * sc, oy,
                           fill="#CBD5E1", outline="#94A3B8")
        c.create_line(ox, oy, ox, shoulder_y, fill="#94A3B8", width=10 * sc)
        a1 = math.radians(angles[0] if angles[0] is not None else 0.0)
        c.create_line(ox, oy - 12 * sc,
                      ox + 34 * sc * math.sin(a1), oy - 12 * sc - 30 * sc * math.cos(a1),
                      fill=JOINT_COLORS[0], width=3, arrow=tk.LAST)
        c.create_text(ox + 34 * sc, oy - 8 * sc, fill=JOINT_COLORS[0],
                      font=FONT_TINY, text="J1 {:+.0f}".format(angles[0] or 0.0))
        # J2 大臂 / J3 小臂（平面串联，屏幕 y 向下取负）
        a2 = math.radians(angles[1] or 0.0)
        p1 = (ox + L1 * math.cos(a2) * sc, shoulder_y - L1 * math.sin(a2) * sc)
        a3 = a2 - math.radians(angles[2] or 0.0)
        p2 = (p1[0] + L2 * math.cos(a3) * sc, p1[1] - L2 * math.sin(a3) * sc)
        for p0, p1_, col in (((ox, shoulder_y), p1, JOINT_COLORS[1]),
                             (p1, p2, JOINT_COLORS[2])):
            c.create_line(p0[0], p0[1], p1_[0], p1_[1], fill="#CBD5E1", width=12 * sc,
                          capstyle=tk.ROUND)
            c.create_line(p0[0], p0[1], p1_[0], p1_[1], fill=col, width=7 * sc,
                          capstyle=tk.ROUND)
        for (x, y), col, idx in (((ox, shoulder_y), JOINT_COLORS[1], 2),
                                 (p1, JOINT_COLORS[2], 3),
                                 (p2, "#EF4444", 0)):
            c.create_oval(x - 6, y - 6, x + 6, y + 6, fill=CARD, outline=col, width=2)
            if idx:
                c.create_text(x + 12, y - 10, fill=col, font=FONT_TINY,
                              text="J{} {:+.0f}".format(idx, angles[idx - 1] or 0.0))
        # J4 选配轴：末端横销随角度旋转
        if self.axis_var.get() == 4:
            a4 = math.radians(angles[3] or 0.0)
            dx, dy = 18 * sc * math.cos(a4), 18 * sc * math.sin(a4)
            c.create_line(p2[0] - dx, p2[1] - dy, p2[0] + dx, p2[1] + dy,
                          fill=JOINT_COLORS[3], width=4)
            c.create_text(p2[0] + 22, p2[1] + 14, fill=JOINT_COLORS[3], font=FONT_TINY,
                          text="J4 {:+.0f}".format(angles[3] or 0.0))
        c.create_text(10, 12, anchor="w", fill=MUTED, font=FONT_SMALL,
                      text="{} 轴模式 · 平面几何示意".format(self.axis_var.get()))

    def _draw_all(self):
        n = self.axis_var.get()
        for i in range(self.profile.max_joints):
            lo, hi = self._limit_at(i)
            self._draw_bar(self.bar_canvases[i],
                           self.last_angles[i] if i < n else None, lo, hi,
                           JOINT_COLORS[i])
        self._draw_trend()
        self._draw_pose(self.last_angles)


__all__ = ["P340Panel", "DEMO_PRESETS", "demo_angles", "pose3d_selftest"]
