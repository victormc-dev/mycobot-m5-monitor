# coding=utf-8
"""myCobot 280 M5 六关节角度实时监控 + 六轴 3D 姿态动画。

用法：
    .venv\\Scripts\\python.exe joint_monitor.py                # 启动界面（自动演示）
    .venv\\Scripts\\python.exe joint_monitor.py --demo          # 强制演示模式
    .venv\\Scripts\\python.exe joint_monitor.py --selftest      # 无界面自检
    .venv\\Scripts\\python.exe joint_monitor.py --model 320     # 新增：适配 myCobot 320 M5

真机模式按型号通过 robot_profiles 工厂创建驱动（280 → MyCobot280，
320 → MyCobot320）读取 get_angles() / get_coords()；默认型号仍为 280 M5。
型号参数、关节限位与界面文案集中在 robot_profiles.py，详见 README_320.md。
演示模式内置四种动作预设，无需硬件即可查看六轴联动动画。

姿态视图用纯 Python 3x3 旋转矩阵构造六自由度串联运动链，正交投影到
tkinter Canvas。轴向按官方 myCobot 280 URDF 的 Wrist Pitch / Roll / Yaw 球腕：
J1 绕世界 Z；J2/J3 肩肘共面俯仰（局部 Y）；J4 腕俯仰（局部 X）；
J5 腕翻滚（局部 Z，连杆轴）；J6 腕偏摆（局部 Y）。
腕部三轴两两垂直，六个关节各画一个彩色轴环，鼠标拖动可改变视角。
该视图是几何示意，用于观察各关节是否参与运动，不是精确 DH 参数解算。
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bootstrap  # noqa: E402  依赖引导（venv 优先，回退 vendor）
import robot_profiles as profiles  # noqa: E402  新增：280 / 320 型号适配层

from pymycobot.mycobot280 import MyCobot280  # noqa: E402  保留原默认驱动导入
from serial.tools import list_ports  # noqa: E402

import tkinter as tk  # noqa: E402
from tkinter import ttk  # noqa: E402

# ---------------------------------------------------------------- 常量
JOINT_NAMES = ("J1 基座旋转", "J2 肩部", "J3 肘部", "J4 腕俯仰", "J5 腕翻滚", "J6 腕偏摆")
JOINT_COLORS = ("#E5484D", "#F59E0B", "#10B981", "#3B82F6", "#8B5CF6", "#EC4899")
COORD_NAMES = ("X", "Y", "Z", "Rx", "Ry", "Rz")
LIMIT = 180.0          # 显示量程 ±180 度（实际限位以设备参数为准）
HISTORY = 120          # 趋势曲线保留帧数
BAR_W, BAR_H = 200, 18         # 仅为初始请求宽度；实际绘制按 winfo_width() 动态取值
TREND_W, TREND_H = 450, 160       # 初始请求尺寸；实际绘制按 winfo_* 动态取值
POSE_W, POSE_H = 450, 320

# 窗口自适应
DESIGN_W, DESIGN_H = 1160, 900   # 设计尺寸（屏幕足够大时采用）
MIN_W, MIN_H = 880, 600          # 最小窗口；更小则主区自动出现纵向滚动条
BODY_MIN_H = 470                 # 主内容自然高度下限
POSE_CHROME = 80                 # 3D 卡片除画布外的固定开销（标题 + 按钮行 + padding）
RIGHT_CHROME = 212               # 右列除趋势画布外的固定开销（实测校准 + 8px 余量）
LEFT_REQ_FULL, LEFT_REQ_SHORT = 700, 580   # 左列自然高度估算（含新增型号选择卡，实测校准）
TOP_PAD, BOTTOM_PAD = 34, 26     # 3D 画布留给顶部指示条 / 底部提示的空间
SIDE_PAD = 10

TIP_FULL = (
    "轴向：J1 基座 / J2 肩 / J3 肘 / J4 腕俯仰 / J5 腕翻滚 / J6 腕偏摆\n"
    "六个彩色圆环标出旋转轴；腕部 J4/J5/J6 三轴两两垂直。\n"
    "「逐关节演示」每 2.5 秒只摆动一个关节，便于逐个观察。\n"
    "几何为串联链示意（含可视化连杆长度），非精确 DH 运动学解算。\n\n"
    "真机接入：外接 12V 电源，Basic 屏进入 Transponder 按 A 显示\n"
    "Atom: OK；读数一直为 -- 时检查 COM 口与波特率 115200。"
)
TIP_SHORT = "J1~J6 全参与，腕部三轴两两垂直；拖动旋转视角、滚轮缩放。非精确 DH 解算。"

BG = "#F4F6F9"
CARD = "#FFFFFF"
TEXT = "#1F2937"
MUTED = "#6B7280"
GRID = "#E5E7EB"
TRACK = "#E9EDF2"
FLOOR = "#DDE3EB"

FONT_UI = ("Microsoft YaHei UI", 10)
FONT_UI_B = ("Microsoft YaHei UI", 10, "bold")
FONT_NUM = ("Consolas", 17, "bold")
FONT_SMALL = ("Microsoft YaHei UI", 9)
FONT_TINY = ("Microsoft YaHei UI", 8)

DEMO_PRESETS = ("自由摆动", "逐关节演示", "抓取循环", "归零静止")

# 姿态几何（可视化用长度单位，非真实 mm）
BASE_H = 30.0
LINK_LEN = (76.0, 64.0, 30.0, 18.0, 16.0)   # 大臂 / 小臂 / 腕根 / 腕中 / 工具（腕部收紧）

# (关节索引, 旋转轴的局部列 0=X/1=Y/2=Z, 旋转轴)。按官方 URDF 的
# Wrist Pitch / Roll / Yaw 球腕，腕部三轴两两垂直；J2/J3 共面是平面臂的正常特性。
JOINT_SPEC = (
    (1, 1, "y"),   # J2 肩俯仰
    (2, 1, "y"),   # J3 肘俯仰
    (3, 0, "x"),   # J4 腕俯仰，垂直于肩肘轴
    (4, 2, "z"),   # J5 腕翻滚，绕连杆轴
    (5, 1, "y"),   # J6 腕偏摆，垂直于 J5
)
RING_R = 13.0
POSE_SCALE = 0.72


# ---------------------------------------------------------------- 3D 数学
def m3mul(a, b):
    return [[a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
             for j in range(3)] for i in range(3)]


def m3col(m, j):
    return [m[0][j], m[1][j], m[2][j]]


def rot_z(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rot_y(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def rot_x(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def vadd(p, v, k=1.0):
    return [p[0] + v[0] * k, p[1] + v[1] * k, p[2] + v[2] * k]


def vcross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def vnorm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 1e-9 else [0.0, 0.0, 1.0]


def ortho_basis(axis):
    """返回垂直于 axis 的两个单位基向量。"""
    u = vnorm(axis)
    ref = [0.0, 0.0, 1.0] if abs(u[2]) < 0.9 else [1.0, 0.0, 0.0]
    v = vnorm(vcross(u, ref))
    w = vnorm(vcross(u, v))
    return v, w


def build_arm(angles, geom=None):
    """按 J1..J6 顺序构造串联运动链，返回全部 3D 图元所需数据。

    geom 为 (base_h, link_len, grid_extent)，由 profiles.scale_geometry() 生成；
    传 None 时沿用 280 M5 的原始可视化尺寸（完全兼容适配前行为）。
    """
    a = [float(x) for x in angles[:6]]
    base_h, link_len, grid_extent = geom if geom is not None else (BASE_H, LINK_LEN, 96.0)
    segments = []      # (p0, p1, joint_index)  joint_index=0 为固定基座立柱
    joints = []        # (center, axis_vector, joint_index)
    frames = []        # 每个关节旋转后的局部坐标系

    p = [0.0, 0.0, 0.0]
    top = [0.0, 0.0, base_h]
    segments.append((p, top, 0))
    p = top

    # J1：基座绕世界 Z 旋转
    joints.append((p, [0.0, 0.0, 1.0], 0))
    R = rot_z(a[0])
    frames.append(R)
    d0 = m3col(R, 0)                     # 随 J1 转动的底座指针方向
    base_pointer = [d0[0] * 30.0, d0[1] * 30.0, 6.0]

    # J2..J6 按 JOINT_SPEC 依次累加旋转并驱动一段连杆（连杆始终沿局部 Z）
    rot_of = {"x": rot_x, "y": rot_y, "z": rot_z}
    for k, col, which in JOINT_SPEC:
        joints.append((p, m3col(R, col), k))
        R = m3mul(R, rot_of[which](a[k]))
        frames.append(R)
        nxt = vadd(p, m3col(R, 2), link_len[k - 1])
        segments.append((p, nxt, k))
        p = nxt

    # J5 关节点处的横销（沿局部 X，随 J5 翻滚转动）；J6 之后的夹爪显示腕偏摆
    cross_at = segments[4][0]
    cx = m3col(frames[4], 0)
    cross_bar = (vadd(cross_at, cx, -8.0), vadd(cross_at, cx, 8.0))
    ex, ez = m3col(R, 0), m3col(R, 2)
    fingers = []
    for sign in (-1.0, 1.0):
        root = vadd(p, ex, sign * 7.0)
        fingers.append((root, vadd(root, ez, 14.0)))

    return {
        "segments": segments,
        "joints": joints,
        "tip": p,
        "base_h": base_h,
        "grid_extent": grid_extent,
        "frame": R,
        "base_pointer": base_pointer,
        "cross_bar": cross_bar,
        "fingers": fingers,
    }


def project(point, az, el, scale, ox, oy):
    """正交投影：返回 (screen_x, screen_y, depth)，depth 越大越远。"""
    ca, sa = math.cos(az), math.sin(az)
    x1 = ca * point[0] - sa * point[1]
    y1 = sa * point[0] + ca * point[1]
    z1 = point[2]
    ce, se = math.cos(el), math.sin(el)
    sy = z1 * ce - y1 * se
    depth = y1 * ce + z1 * se
    return ox + x1 * scale, oy - sy * scale, depth


def pose_points(arm):
    """姿态视图中所有需要保证可见的 3D 点（含地面网格四角）。"""
    pts = []
    for p0, p1, _ in arm["segments"]:
        pts.append(p0)
        pts.append(p1)
    for center, _axis, _idx in arm["joints"]:
        pts.append(center)
    pts.extend(arm["cross_bar"])
    for root, tip in arm["fingers"]:
        pts.append(root)
        pts.append(tip)
    pts.append(arm["base_pointer"])
    pts.append(arm["tip"])
    g = arm.get("grid_extent", 96.0)
    for x in (-g, g):
        for y in (-g, g):
            pts.append([x, y, 0.0])
    return pts


def lerp_angles(cur, tgt, factor=0.22):
    """角度插值：取最小角差，避免 ±180 边界绕远路。返回 (新角度, 是否仍在移动)。"""
    out = list(cur)
    moving = False
    for i in range(6):
        delta = (tgt[i] - out[i] + 180.0) % 360.0 - 180.0
        if abs(delta) > 0.02:
            v = out[i] + delta * factor
            out[i] = (v + 180.0) % 360.0 - 180.0   # 规范到 (-180, 180]
            moving = True
        else:
            out[i] = tgt[i]
    return out, moving


# ---------------------------------------------------------------- 数据源
class AngleSource:
    """统一真机 / 演示两种数据源。"""

    def __init__(self, port, baud, demo, preset=DEMO_PRESETS[0], profile=None):
        self.port = port
        self.baud = baud
        self.demo = demo
        self.preset = preset
        self.profile = profile or profiles.get_profile(profiles.DEFAULT_MODEL)
        self.mc = None
        self._n = 0
        self._coords = None
        self.active = -1        # 当前演示关节索引，-1 表示无高亮

    def open(self):
        if self.demo:
            return
        if not self.port:
            raise ValueError("未选择串口，请先点击「刷新」扫描端口")
        self.mc = profiles.create_robot(self.profile, self.port, self.baud)

    @staticmethod
    def demo_angles(t, preset):
        """返回 (六轴角度, 当前活动关节索引)。全部预设均驱动 J1..J6。"""
        if preset == "归零静止":
            return [0.0] * 6, -1

        if preset == "逐关节演示":
            slot = int(t / 2.5)
            idx = slot % 6
            phase = (t - slot * 2.5) / 2.5
            base = [0.0, -30.0, 65.0, -35.0, 0.0, 0.0]
            base[idx] += 55.0 * math.sin(2.0 * math.pi * phase)
            return [round(x, 2) for x in base], idx

        if preset == "抓取循环":
            w = 2.0 * math.pi * (t % 8.0) / 8.0
            vals = [
                48.0 * math.sin(w),
                -22.0 + 26.0 * math.cos(w),
                52.0 + 20.0 * math.sin(w + 0.6),
                -32.0 + 18.0 * math.cos(w + 1.1),
                55.0 * math.sin(2.0 * w),
                85.0 * math.sin(w + 0.9),
            ]
            return [round(x, 2) for x in vals], -1

        # 自由摆动：六个关节不同频率 + 直流偏置，零位之外呈自然弯折姿态
        vals = [
            60.0 * math.sin(0.42 * t),
            -28.0 + 30.0 * math.sin(0.55 * t + 0.7),
            62.0 + 26.0 * math.sin(0.70 * t + 1.4),
            -32.0 + 22.0 * math.sin(0.85 * t + 2.1),
            70.0 * math.sin(0.63 * t + 2.8),
            100.0 * math.sin(0.38 * t + 3.5),
        ]
        return [round(x, 2) for x in vals], -1

    def read(self):
        """返回 (angles, coords, elapsed_ms, active_joint)。"""
        t0 = time.perf_counter()
        if self.demo:
            angles, self.active = self.demo_angles(time.monotonic(), self.preset)
            return angles, None, (time.perf_counter() - t0) * 1000.0, self.active

        self.active = -1
        self._n += 1
        angles = self.mc.get_angles()
        coords = None
        if self._n % 5 == 1:                 # 坐标读取降频，减少串口往返
            try:
                coords = self.mc.get_coords()
            except Exception:
                coords = None
        if coords:
            self._coords = coords
        ms = (time.perf_counter() - t0) * 1000.0
        if not angles or len(angles) < 6:    # 无响应时保留上一帧
            return None, self._coords, ms, -1
        return [float(x) for x in angles[:6]], self._coords, ms, -1

    def close(self):
        try:
            if self.mc is not None:
                self.mc._serial_port.close()
        except Exception:
            pass
        self.mc = None


class Reader(threading.Thread):
    """后台采样线程：串口读取不阻塞 UI。"""

    def __init__(self, source, interval_ms, out):
        super().__init__(daemon=True)
        self.source = source
        self.interval = interval_ms / 1000.0
        self.out = out
        self.stop_evt = threading.Event()

    def run(self):
        try:
            self.source.open()
        except Exception as exc:
            self.out.put(("error", "{}: {}".format(type(exc).__name__, exc)))
            self.out.put(("closed", None))
            return
        self.out.put(("ready", None))
        try:
            while not self.stop_evt.is_set():
                try:
                    self.out.put(("data", self.source.read()))
                except Exception as exc:
                    self.out.put(("error", "{}: {}".format(type(exc).__name__, exc)))
                    break
                self.stop_evt.wait(self.interval)
        finally:
            self.source.close()
            self.out.put(("closed", None))


# ---------------------------------------------------------------- 界面
class MonitorApp(tk.Tk):
    def __init__(self, force_demo=False, model=profiles.DEFAULT_MODEL):
        super().__init__()
        self.profile = profiles.get_profile(model)
        self.model_var = tk.StringVar(value=self.profile.key)   # 选择卡 / 状态栏下拉共用
        self.p340_view = None                                   # 四轴面板，按需创建
        self.title(self._window_title())
        # 初始几何按屏幕工作区自适应，且不超过设计尺寸
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w = max(760, min(DESIGN_W, int(sw * 0.94), sw - 40))
        h = max(560, min(DESIGN_H, int((sh - 80) * 0.96), sh - 60))
        self.geometry("{}x{}+{}+{}".format(w, h, max(0, (sw - w) // 2), max(0, (sh - h) // 3)))
        self.minsize(MIN_W, MIN_H)
        self.configure(bg=BG)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.q = queue.Queue()
        self.reader = None
        self.running = False
        self.frames = 0
        self.last_ms = 0.0
        self.history = []
        self.last_angles = [0.0] * 6
        self.target_angles = [0.0] * 6     # 最新采样值
        self.pose_angles = [0.0] * 6       # 3D 视图插值后的显示值
        self.active_joint = -1

        # 视角状态
        self.view_az = math.radians(-35.0)
        self.view_el = math.radians(24.0)
        self.view_zoom = 1.0
        self._drag = None
        self.auto_fit = 1.0          # 3D 自动缩放系数（平滑跟随）
        self._fit_snap = False       # resize 时跳过平滑，立即贴合
        self._resize_job = None

        self._build_toolbar()
        self._build_body()
        self._build_status()
        if self.profile.key == "p340":       # 命令行直接指定 P340
            self._switch_view(self.profile)
        self.bind("<Configure>", self._on_resize)
        self.scan_ports()
        self._draw_pose(self.pose_angles)          # 零位立即可见
        self.after(40, self._poll)
        self.after(33, self._animate)              # 3D 视图独立 ~30 FPS 插值
        if force_demo or self.demo_var.get():
            self.demo_var.set(True)
            self.after(300, self._auto_start_demo)

    def _window_title(self):
        suffix = ("四轴码垛臂监控" if self.profile.key == "p340"
                  else "六关节角度实时监控 / 六轴姿态动画")
        return self.profile.label + "  " + suffix

    def _switch_view(self, prof):
        """P340 切到独立四轴面板；280/320 用原六轴视图。

        仅替换 Canvas window 项，六轴渲染、数据源、演示预设一律不动。
        """
        if prof.key == "p340":
            if self.p340_view is None:
                import p340_view
                self.p340_view = p340_view.P340Panel(self.host_canvas,
                                                     on_status=self._set_status)
            target = self.p340_view
        else:
            target = self.body
        self.host_canvas.itemconfig(self._body_win, window=target)
        self.host_canvas.configure(scrollregion=(
            0, 0, max(840, self.host_canvas.winfo_width()),
            max(BODY_MIN_H, self.host_canvas.winfo_height())))

    def _auto_start_demo(self):
        if not self.running and self.demo_var.get():
            self._start()

    # ---- 布局
    def _build_toolbar(self):
        """单行紧凑工具栏；尾部弹性列使按钮靠右，窄窗口下控件不互相挤压。"""
        bar = tk.Frame(self, bg=CARD, bd=0, highlightthickness=1, highlightbackground=GRID)
        bar.pack(fill="x", padx=12, pady=(12, 8))
        bar.columnconfigure(10, weight=1)
        self.toolbar = bar

        tk.Label(bar, text="串口", bg=CARD, fg=TEXT, font=FONT_UI_B).grid(
            row=0, column=0, padx=(8, 4), pady=10)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=11,
                                     state="readonly", font=FONT_UI)
        self.port_box.grid(row=0, column=1, padx=4)
        tk.Button(bar, text="刷新", command=self.scan_ports, font=FONT_UI, relief="flat",
                  bg="#EDF1F6", fg=TEXT, padx=4).grid(row=0, column=2, padx=3)

        tk.Label(bar, text="波特率", bg=CARD, fg=TEXT, font=FONT_UI_B).grid(
            row=0, column=3, padx=(8, 4))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(bar, textvariable=self.baud_var, width=7, font=FONT_UI,
                     values=("115200", "1000000", "57600", "9600")).grid(row=0, column=4, padx=4)

        tk.Label(bar, text="采样", bg=CARD, fg=TEXT, font=FONT_UI_B).grid(
            row=0, column=5, padx=(8, 4))
        self.interval_var = tk.StringVar(value="200 ms")
        ttk.Combobox(bar, textvariable=self.interval_var, width=7, state="readonly", font=FONT_UI,
                     values=("100 ms", "200 ms", "500 ms", "1000 ms")).grid(row=0, column=6, padx=4)

        self.demo_var = tk.BooleanVar(value=False)
        tk.Checkbutton(bar, text="演示模式", variable=self.demo_var, bg=CARD, fg=TEXT,
                       font=FONT_UI, activebackground=CARD, selectcolor=CARD,
                       command=self._on_demo_toggle).grid(row=0, column=7, padx=(6, 2))

        tk.Label(bar, text="动作", bg=CARD, fg=TEXT, font=FONT_UI_B).grid(
            row=0, column=8, padx=(6, 4))
        self.preset_var = tk.StringVar(value=DEMO_PRESETS[0])
        cb = ttk.Combobox(bar, textvariable=self.preset_var, width=10, state="readonly",
                          font=FONT_UI, values=DEMO_PRESETS)
        cb.grid(row=0, column=9, padx=4)
        cb.bind("<<ComboboxSelected>>", self._on_preset_change)

        self.run_btn = tk.Button(bar, text="开始采集", command=self._toggle_run, font=FONT_UI_B,
                                 bg="#2563EB", fg="white", relief="flat", padx=12, pady=4)
        self.run_btn.grid(row=0, column=11, padx=(6, 8), pady=8, sticky="e")

    def _build_body(self):
        # 滚动宿主：窗口够大时内容自动铺满，窗口过矮时自动出现纵向滚动条
        host = tk.Frame(self, bg=BG)
        host.pack(fill="both", expand=True, padx=12, pady=4)
        hc = tk.Canvas(host, bg=BG, highlightthickness=0)
        vsb = ttk.Scrollbar(host, orient="vertical", command=hc.yview)
        hc.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        hc.pack(side="left", fill="both", expand=True)
        body = tk.Frame(hc, bg=BG)
        self._body_win = hc.create_window((0, 0), window=body, anchor="nw")
        self.host_canvas, self.vsb, self.body = hc, vsb, body
        self._vsb_visible = True
        hc.bind("<Configure>", self._on_host_configure)
        hc.bind("<MouseWheel>", lambda e: hc.yview_scroll(int(-e.delta / 120), "units"))

        # 三列：关节数值 | 3D 几何演示（独立主纵列） | 趋势 + 坐标
        body.grid_columnconfigure(0, weight=1, minsize=225)
        body.grid_columnconfigure(1, weight=6, minsize=380)
        body.grid_columnconfigure(2, weight=2, minsize=235)
        body.grid_rowconfigure(0, weight=1, minsize=BODY_MIN_H)

        left = tk.Frame(body, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.left_frame = left

        # 新增：机械臂型号选择卡（单行紧凑，280 / 320 / P340）
        card = tk.Frame(left, bg="#F8FAFC", highlightthickness=1, highlightbackground=GRID)
        card.pack(fill="x", padx=10, pady=(8, 2))
        tk.Label(card, text="型号", bg="#F8FAFC", fg=MUTED,
                 font=FONT_SMALL).pack(side="left", padx=(6, 3), pady=4)
        short = {"280": "280 M5", "320": "320 M5", "p340": "P340"}
        for key in profiles.model_keys():
            tk.Radiobutton(card, text=short.get(key, key), value=key,
                           variable=self.model_var, indicatoron=False,
                           command=self._on_model_change, bg="#EDF1F6",
                           selectcolor="#DBEAFE", fg=TEXT, font=FONT_TINY,
                           relief="flat", padx=4, pady=1).pack(side="left", padx=1, pady=3)

        tk.Label(left, text="六关节角度  ( 单位: 度 )", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=14, pady=(12, 2))
        self.limit_caption = tk.Label(
            left, text=self.profile.limit_caption or "显示量程 ±180；实际限位以设备参数为准",
            bg=CARD, fg=MUTED, font=FONT_SMALL, justify="left", anchor="w", wraplength=200)
        self.limit_caption.pack(anchor="w", padx=14, pady=(0, 6))

        self.value_labels = []
        self.bar_canvases = []
        for i, name in enumerate(JOINT_NAMES):
            row = tk.Frame(left, bg=CARD)
            row.pack(fill="x", padx=14, pady=6)
            head = tk.Frame(row, bg=CARD)
            head.pack(fill="x")
            tk.Label(head, text=name, bg=CARD, fg=JOINT_COLORS[i], font=FONT_UI_B).pack(side="left")
            v = tk.Label(head, text="--.--", bg=CARD, fg=TEXT, font=FONT_NUM, width=9, anchor="e")
            v.pack(side="right")
            self.value_labels.append(v)
            c = tk.Canvas(row, width=BAR_W, height=BAR_H, bg=TRACK, highlightthickness=0)
            c.pack(fill="x", pady=(4, 0))
            self.bar_canvases.append(c)

        tk.Label(left, bg=CARD, fg=TEXT, font=FONT_UI_B,
                 text="六轴姿态动画说明").pack(anchor="w", padx=14, pady=(20, 4))
        self.tip_label = tk.Label(left, bg=CARD, fg=MUTED, font=FONT_SMALL,
                                  justify="left", anchor="w", wraplength=200,
                                  text=self.profile.tip_full or TIP_FULL)
        self.tip_label.pack(anchor="w", padx=14, pady=(0, 12))
        tk.Frame(left, bg=CARD).pack(fill="both", expand=True)   # 多余高度留白

        # 中列：3D 几何演示独占一个纵列，高度铺满整个主区
        center = tk.Frame(body, bg=BG)
        center.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        center.grid_columnconfigure(0, weight=1)
        center.grid_rowconfigure(0, weight=0)
        center.grid_rowconfigure(1, weight=1)   # 多余高度留白
        self.center_frame = center

        fpose = tk.Frame(center, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        fpose.grid(row=0, column=0, sticky="new")
        tk.Label(fpose, text="六轴 3D 几何演示（J1-J6 全参与）", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=12, pady=(8, 2))
        self.pose = tk.Canvas(fpose, width=280, height=POSE_H, bg=CARD, highlightthickness=0)
        self.pose.pack(fill="x", padx=12, pady=(0, 2))
        self.pose.bind("<ButtonPress-1>", self._pose_press)
        self.pose.bind("<B1-Motion>", self._pose_drag)
        self.pose.bind("<MouseWheel>", self._pose_wheel)
        self.pose.bind("<Double-Button-1>", lambda e: self._pose_reset())
        btnrow = tk.Frame(fpose, bg=CARD)
        btnrow.pack(fill="x", padx=12, pady=(0, 8))
        tk.Label(btnrow, text="拖动旋转 / 滚轮缩放 / 双击复位", bg=CARD, fg=MUTED,
                 font=FONT_SMALL).pack(side="left")
        tk.Button(btnrow, text="复位视角", command=self._pose_reset, font=FONT_SMALL,
                  relief="flat", bg="#EDF1F6", fg=TEXT, padx=10).pack(side="right")

        # 右列：趋势 + 末端坐标
        right = tk.Frame(body, bg=BG)
        right.grid(row=0, column=2, sticky="nsew")
        self.right_frame = right
        self._tip_compact = False
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(0, weight=0)
        right.grid_rowconfigure(1, weight=0)
        right.grid_rowconfigure(2, weight=1)   # 多余高度留白，内容顶部对齐

        f1 = tk.Frame(right, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        f1.grid(row=0, column=0, sticky="new", pady=(0, 8))
        tk.Label(f1, text="角度趋势（最近 {} 帧）".format(HISTORY), bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=12, pady=(8, 2))
        self.trend = tk.Canvas(f1, width=220, height=TREND_H, bg=CARD, highlightthickness=0)
        self.trend.pack(fill="x", padx=12, pady=(0, 2))
        legend = tk.Frame(f1, bg=CARD)
        legend.pack(pady=(0, 8))
        for i in range(6):
            tk.Label(legend, text="J{}".format(i + 1), bg=CARD, fg=JOINT_COLORS[i],
                     font=FONT_SMALL).pack(side="left", padx=6)

        f3 = tk.Frame(right, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        f3.grid(row=1, column=0, sticky="new")
        self.f3 = f3
        tk.Label(f3, text="末端坐标（真机，每 5 帧刷新）", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=12, pady=(8, 2))
        self.coord_labels = []
        grid = tk.Frame(f3, bg=CARD)
        grid.pack(fill="x", padx=12, pady=(0, 8))
        for col in range(4):
            grid.columnconfigure(col, weight=1 if col % 2 else 0)
        for i, nm in enumerate(COORD_NAMES):
            tk.Label(grid, text=nm, bg=CARD, fg=MUTED, font=FONT_SMALL, width=2).grid(
                row=i // 2, column=(i % 2) * 2, padx=(0, 2), pady=2, sticky="w")
            lab = tk.Label(grid, text="--", bg="#F3F5F8", fg=TEXT, font=("Consolas", 10),
                           width=8, anchor="e")
            lab.grid(row=i // 2, column=(i % 2) * 2 + 1, padx=(0, 8), pady=2, sticky="ew")
            self.coord_labels.append(lab)

    def _build_status(self):
        bar = tk.Frame(self, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        bar.pack(fill="x", padx=12, pady=(4, 12))
        self.status = tk.Label(bar, text="就绪 · 未开始采集", bg=CARD, fg=MUTED,
                               font=FONT_SMALL, anchor="w")
        self.status.pack(side="left", padx=12, pady=7)
        self.src_label = tk.Label(bar, text="依赖: " + bootstrap.SOURCE, bg=CARD, fg=MUTED,
                                  font=FONT_SMALL)
        self.src_label.pack(side="right", padx=12)

        # 新增：型号切换放在状态栏，不影响原有单行工具栏布局
        tk.Label(bar, text="型号", bg=CARD, fg=MUTED, font=FONT_SMALL).pack(
            side="right", padx=(0, 4))
        self.model_box = ttk.Combobox(bar, textvariable=self.model_var, width=6,
                                      state="readonly", font=FONT_SMALL,
                                      values=profiles.model_keys())
        self.model_box.pack(side="right", padx=(0, 8))
        self.model_box.bind("<<ComboboxSelected>>", self._on_model_change)

    # ---- 尺寸自适应
    def _on_host_configure(self, evt):
        """3D 演示独占纵列并铺满主区高度；窗口过矮时保持自然高度并启用滚动。"""
        w = max(evt.width, 840)          # 三列 minsize 合计
        avail = max(300, evt.height)
        h_pose = max(220, avail - POSE_CHROME)
        h_trend = max(70, avail - RIGHT_CHROME)   # 右列同样铺满，三列底边对齐
        self.pose.configure(height=h_pose)
        self.trend.configure(height=h_trend)
        compact = (evt.height < 620) or (self.left_frame.winfo_width() < 320)
        if compact != self._tip_compact:
            self._tip_compact = compact
            self.tip_label.configure(
                text=(self.profile.tip_short or TIP_SHORT) if compact
                else (self.profile.tip_full or TIP_FULL))
        # 新增：用左列真实请求高度兜底（型号选择卡加入后左列内容更高）
        need_h = max(BODY_MIN_H, POSE_CHROME + h_pose, RIGHT_CHROME + h_trend,
                     LEFT_REQ_SHORT if compact else LEFT_REQ_FULL,
                     self.left_frame.winfo_reqheight())
        h = max(evt.height, need_h)
        self.host_canvas.itemconfig(self._body_win, width=w, height=h)
        self.host_canvas.configure(scrollregion=(0, 0, w, h))
        show = need_h > evt.height + 2
        if show != self._vsb_visible:
            if show:
                self.vsb.pack(side="right", fill="y", before=self.host_canvas)
            else:
                self.vsb.pack_forget()
            self._vsb_visible = show

    def _on_resize(self, evt):
        if evt.widget is not self:      # 只响应顶层窗口尺寸变化
            return
        if self._resize_job is not None:
            try:
                self.after_cancel(self._resize_job)
            except Exception:
                pass
        self._resize_job = self.after(80, self._after_resize)   # 防抖

    def _after_resize(self):
        self._resize_job = None
        # 说明文字保持固定 wraplength：动态加大反而会撑宽左列、压缩 3D 主列
        self._fit_snap = True           # 立即贴合新画布，避免平滑滞后造成裁切
        self._draw_trend()
        if self.frames > 0:
            for i, cv in enumerate(self.bar_canvases):
                self._draw_bar(cv, self.last_angles[i], JOINT_COLORS[i],
                               self.profile.display_limit)
        self._draw_pose(self.pose_angles, self.active_joint)
        self._fit_snap = False

        # 实测兜底：布局稳定后按控件真实请求高度校正，杜绝估算偏差造成的裁切
        try:
            req = max(BODY_MIN_H, self.body.winfo_reqheight())
            host_h = self.host_canvas.winfo_height()
            if req > host_h + 2:
                self.host_canvas.itemconfig(self._body_win, height=req)
                w = max(self.host_canvas.winfo_width(), 700)
                self.host_canvas.configure(scrollregion=(0, 0, w, req))
                if not self._vsb_visible:
                    self.vsb.pack(side="right", fill="y", before=self.host_canvas)
                    self._vsb_visible = True
        except Exception:
            pass

    # ---- 交互
    def scan_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports:
            if self.port_var.get() not in ports:
                self.port_var.set(ports[0])
        else:
            self.port_var.set("")
            if not self.running:
                self.demo_var.set(True)       # 没有硬件时自动切到演示模式
        self._set_status("扫描到 {} 个串口".format(len(ports)) if ports else "未发现串口，已切换到演示模式")

    def _on_demo_toggle(self):
        if self.demo_var.get():
            self._set_status("演示模式：使用模拟数据，不连接串口")
        else:
            self._set_status("真机模式：请确认 COM 口与 115200 波特率")

    def _on_preset_change(self, _evt=None):
        if self.profile.key == "p340":
            if self.p340_view is not None:
                self.p340_view.set_preset(self.preset_var.get())
            else:
                self._set_status("演示动作切换为：" + self.preset_var.get())
            return
        if self.reader is not None and self.running:
            self.reader.source.preset = self.preset_var.get()
        self._set_status("演示动作切换为：" + self.preset_var.get())

    def _on_model_change(self, _evt=None):
        """新增：切换 280 / 320 型号；采集中禁止切换以免驱动与参数错配。"""
        key = self.model_var.get()
        if self.running:
            self.model_var.set(self.profile.key)
            self._set_status("采集中不能切换型号，请先点击「停止采集」")
            return
        try:
            prof = profiles.get_profile(key)
        except ValueError as exc:
            self.model_var.set(self.profile.key)
            self._set_status(str(exc))
            return
        self.profile = prof
        self.title(self._window_title())
        self._switch_view(prof)
        self.limit_caption.configure(
            text=prof.limit_caption or "显示量程 ±180；实际限位以设备参数为准")
        self.tip_label.configure(
            text=(prof.tip_short or TIP_SHORT) if self._tip_compact
            else (prof.tip_full or TIP_FULL))
        self.baud_var.set(str(prof.baud))
        self._fit_snap = True
        self._draw_pose(self.pose_angles, self.active_joint)
        self._draw_trend()
        if self.frames > 0:
            for i, cv in enumerate(self.bar_canvases):
                self._draw_bar(cv, self.last_angles[i], JOINT_COLORS[i], prof.display_limit)
        self._fit_snap = False
        self._set_status("型号已切换：{} | {}{}".format(
            prof.label, profiles.summary(prof),
            " | 四轴专用面板" if prof.key == "p340" else ""))

    def _toggle_run(self):
        if self.running:
            self._stop()
        else:
            self._start()

    def _start(self):
        demo = self.demo_var.get()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self._set_status("波特率必须是数字")
            return
        interval = int(str(self.interval_var.get()).split()[0])
        if self.profile.key == "p340":
            if self.p340_view is None:
                self._switch_view(self.profile)
            self.p340_view.start("" if demo else self.port_var.get(), baud, demo, interval,
                                 preset=self.preset_var.get())
            self.running = True
            self.frames = 0
            self.history = []
            self.run_btn.config(text="停止采集", bg="#DC2626")
            return
        src = AngleSource("" if demo else self.port_var.get(), baud, demo,
                          self.preset_var.get(), profile=self.profile)
        self.reader = Reader(src, interval, self.q)
        self.running = True
        self.frames = 0
        self.history = []
        self.run_btn.config(text="停止采集", bg="#DC2626")
        self._set_status("连接中..." if not demo else "演示模式采集中...")
        self.reader.start()

    def _stop(self):
        if self.reader is not None:
            self.reader.stop_evt.set()
        if self.p340_view is not None:
            self.p340_view.stop()
        self.running = False
        self.run_btn.config(text="开始采集", bg="#2563EB")
        self._set_status("已停止采集")

    def _set_status(self, text):
        self.status.config(text=text)

    # ---- 3D 视角交互
    def _pose_press(self, evt):
        self._drag = (evt.x, evt.y, self.view_az, self.view_el)

    def _pose_drag(self, evt):
        if not self._drag:
            return
        x0, y0, az0, el0 = self._drag
        self.view_az = az0 + (evt.x - x0) * 0.010
        self.view_el = max(-0.25, min(1.30, el0 + (evt.y - y0) * 0.007))
        self._draw_pose(self.pose_angles, self.active_joint)

    def _pose_wheel(self, evt):
        self.view_zoom = max(0.55, min(2.0, self.view_zoom * (1.10 if evt.delta > 0 else 0.90)))
        self._draw_pose(self.pose_angles, self.active_joint)

    def _pose_reset(self):
        self.view_az = math.radians(-35.0)
        self.view_el = math.radians(24.0)
        self.view_zoom = 1.0
        self._draw_pose(self.pose_angles, self.active_joint)

    # ---- 数据轮询
    def _poll(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "ready":
                    self._set_status("采集中...")
                elif kind == "error":
                    self._set_status("错误: " + str(payload))
                    self.running = False
                    self.run_btn.config(text="开始采集", bg="#2563EB")
                elif kind == "closed":
                    self.running = False
                    self.run_btn.config(text="开始采集", bg="#2563EB")
                elif kind == "data" and self.running:
                    self._update(*payload)
        except queue.Empty:
            pass
        if (self.profile.key == "p340" and self.p340_view is not None
                and self.running and not self.p340_view.running):
            self.running = False
            self.run_btn.config(text="开始采集", bg="#2563EB")
        self.after(40, self._poll)

    def _animate(self):
        """3D 姿态独立于串口采样率插值重绘，避免低速采样造成画面卡顿。"""
        new_angles, moving = lerp_angles(self.pose_angles, self.target_angles)
        self.pose_angles = new_angles
        if self.running or moving:
            self.last_angles = self.pose_angles[:]
            if self.profile.key != "p340":
                self._draw_pose(self.pose_angles, self.active_joint)
        self.after(33, self._animate)

    def _update(self, angles, coords, elapsed_ms, active=-1):
        self.last_ms = elapsed_ms
        self.frames += 1
        self.active_joint = active
        if angles is None:
            self._set_status("警告: 机械臂无响应（检查 {} / 供电 / {} 波特率）".format(
                "串口转发" if self.profile.key == "320" else "Transponder",
                self.profile.baud))
            return
        self.target_angles = list(angles)
        self.history.append(angles)
        if len(self.history) > HISTORY:
            self.history.pop(0)

        for i, val in enumerate(angles):
            self.value_labels[i].config(text="{:+.2f}".format(val))
            self._draw_bar(self.bar_canvases[i], val, JOINT_COLORS[i],
                           self.profile.display_limit)
        self._draw_trend()
        # 3D 姿态由 _animate 以 ~30 FPS 插值绘制，此处不直接重绘

        if coords:
            for i, lab in enumerate(self.coord_labels):
                try:
                    lab.config(text="{:.1f}".format(float(coords[i])))
                except (IndexError, TypeError, ValueError):
                    lab.config(text="--")
        mode = "演示" if self.demo_var.get() else "真机 " + self.port_var.get()
        extra = " | 演示 J{}".format(active + 1) if active >= 0 else ""
        over = profiles.over_limit(self.profile, angles)
        warn = " | 超限位 J{}".format(",".join(str(j) for j in over)) if over else ""
        self._set_status("{} | {} | 采样 {} | 帧 {} | 读取 {:.1f} ms{}{}".format(
            mode, self.preset_var.get(), self.interval_var.get(), self.frames,
            elapsed_ms, extra, warn))

    # ---- 2D 绘图
    @staticmethod
    def _draw_bar(c, value, color, limit=LIMIT):
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10:
            w = BAR_W
        if h < 6:
            h = BAR_H
        c.create_rectangle(0, 0, w, h, fill=TRACK, outline="")
        mid = w / 2
        frac = max(-1.0, min(1.0, value / limit))
        x = mid + frac * (mid - 3)
        x0, x1 = (mid, x) if x >= mid else (x, mid)
        if x1 - x0 > 0.5:
            c.create_rectangle(x0, 2, x1, h - 2, fill=color, outline="")
        c.create_line(mid, 0, mid, h, fill="#AAB2BF")

    def _draw_trend(self):
        c = self.trend
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 20:
            w = TREND_W
        if h < 20:
            h = TREND_H
        pad_l, pad_r, pad_t, pad_b = 36, 10, 12, 20
        plot_h = max(20, h - pad_t - pad_b)
        plot_w = max(20, w - pad_l - pad_r)

        lim = self.profile.display_limit

        def y_of(v):
            frac = max(-1.0, min(1.0, v / lim))
            return pad_t + (1.0 - frac) / 2.0 * plot_h

        # 网格密度随画布高度自适应，拉高后不会中间空荡
        n_grid = 9 if h >= 320 else (7 if h >= 200 else 5)
        for i in range(n_grid):
            v = lim - 2.0 * lim * i / (n_grid - 1)
            y = y_of(v)
            c.create_line(pad_l, y, w - pad_r, y, fill=GRID)
        c.create_line(pad_l, y_of(0.0), w - pad_r, y_of(0.0), fill="#C7CDD6", width=1.4)
        for v in (lim, lim / 2.0, 0.0, -lim / 2.0, -lim):
            c.create_text(pad_l - 5, y_of(v), anchor="e", fill=MUTED, font=FONT_TINY,
                          text=("{:+d}".format(int(v)) if v else "0"))
        c.create_text(pad_l, h - 8, anchor="w", fill=MUTED, font=FONT_TINY,
                      text="-{} 帧".format(HISTORY))
        c.create_text(w - pad_r, h - 8, anchor="e", fill=MUTED, font=FONT_TINY, text="现在")

        if len(self.history) < 2:
            return
        for j in range(6):
            pts = []
            for i, row in enumerate(self.history):
                x = pad_l + (i / max(1, HISTORY - 1)) * plot_w
                pts.extend((x, y_of(row[j])))
            c.create_line(*pts, fill=JOINT_COLORS[j], width=1.6)

    # ---- 六轴 3D 姿态
    def _fit_pose(self, arm, W, H, az, el):
        """按当前姿态所有 3D 点的包围盒自动求缩放与居中，保证机械臂完整可见。"""
        pts = pose_points(arm)              # 含地面网格四角，避免背景被裁

        # scale=1、无偏移的原始投影坐标；最终 screen = (ox + rx*sc, oy + ry*sc)
        raw = [project(p, az, el, 1.0, 0.0, 0.0) for p in pts]
        xs = [r[0] for r in raw]
        ys = [r[1] for r in raw]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
        spanx = max(1.0, maxx - minx)
        spany = max(1.0, maxy - miny)
        avail_w = max(30.0, W - 2 * SIDE_PAD)
        avail_h = max(30.0, H - TOP_PAD - BOTTOM_PAD)
        target = min(avail_w / spanx, avail_h / spany) * 0.92
        target = max(0.01, min(2.50, target))   # 上限只会缩小、不会造成越界
        if self._fit_snap:
            self.auto_fit = target              # resize 时立即贴合
        else:
            self.auto_fit += (target - self.auto_fit) * 0.18   # 平滑，避免画面呼吸
        sc = self.auto_fit * self.view_zoom
        cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
        ox = W / 2.0 - cx * sc
        oy = TOP_PAD + avail_h / 2.0 - cy * sc
        return sc, ox, oy

    def _draw_pose(self, a, active=-1):
        c = self.pose
        c.delete("all")
        W, H = c.winfo_width(), c.winfo_height()
        if W < 40:
            W = POSE_W
        if H < 40:
            H = POSE_H
        az, el = self.view_az, self.view_el
        geom = profiles.scale_geometry(BASE_H, LINK_LEN, 96.0, self.profile.geom_scale)
        arm = build_arm(a, geom)
        sc, ox, oy = self._fit_pose(arm, W, H, az, el)
        self._last_pose_transform = (sc, ox, oy)
        self._last_pose_size = (W, H)

        def P(v):
            return project(v, az, el, sc, ox, oy)

        items = []                     # (depth, seq, draw_callable)

        def add_line(p0, p1, color, width, dash=None):
            s0, s1 = P(p0), P(p1)
            d = (s0[2] + s1[2]) / 2.0
            items.append((d, len(items),
                          lambda s0=s0, s1=s1, color=color, width=width, dash=dash:
                          c.create_line(s0[0], s0[1], s1[0], s1[1], fill=color,
                                        width=width, capstyle=tk.ROUND, dash=dash)))

        def add_poly(points, color, width):
            sp = [P(v) for v in points]
            d = sum(v[2] for v in sp) / len(sp)
            flat = [x for v in sp for x in (v[0], v[1])]
            items.append((d, len(items),
                          lambda flat=flat, color=color, width=width:
                          c.create_line(*flat, fill=color, width=width, capstyle=tk.ROUND)))

        def add_dot(p3, r, fill, outline, width=1):
            s = P(p3)
            items.append((s[2], len(items),
                          lambda s=s, r=r, fill=fill, outline=outline, width=width:
                          c.create_oval(s[0] - r, s[1] - r, s[0] + r, s[1] + r,
                                        fill=fill, outline=outline, width=width)))

        # 地面参考网格：J1 回转时提供静止参照；尺寸随型号工作半径缩放
        g = arm["grid_extent"]
        for k in range(-2, 3):
            t = k * g * 46.0 / 96.0
            add_line([t, -g, 0.0], [t, g, 0.0], FLOOR, 1)
            add_line([-g, t, 0.0], [g, t, 0.0], FLOOR, 1)

        # 底座 + 随 J1 旋转的指针
        add_dot([0, 0, 0], 30, "#CBD5E1", "#94A3B8", 1)
        add_line([0, 0, 0], [0, 0, arm["base_h"]], "#94A3B8", 12)
        add_line([0, 0, 6.0], arm["base_pointer"], JOINT_COLORS[0], 3)

        # 六段连杆（外描边 + 内芯，形成柱体观感）
        for p0, p1, idx in arm["segments"]:
            col = "#94A3B8" if idx == 0 else JOINT_COLORS[idx]
            add_line(p0, p1, "#CBD5E1", 12)
            add_line(p0, p1, col, 7)

        # J5 横销、J6 夹爪：让两个绕轴自转的关节在画面里可见
        add_line(arm["cross_bar"][0], arm["cross_bar"][1], JOINT_COLORS[4], 5)
        for root, tip in arm["fingers"]:
            add_line(root, tip, JOINT_COLORS[5], 5)

        # 六个关节各自的旋转轴环（垂直于该关节旋转轴）
        for center, axis, idx in arm["joints"]:
            v, w = ortho_basis(axis)
            r = (RING_R if idx < 4 else RING_R * 0.62) + (2.0 if idx == active else 0.0)
            ring = [vadd(vadd(center, v, r * math.cos(t * math.pi / 12.0)),
                         w, r * math.sin(t * math.pi / 12.0))
                    for t in range(24)]
            lw = 3.5 if idx == active else 2.0
            add_poly(ring + [ring[0]], JOINT_COLORS[idx], lw)

        # 关节点
        for center, _axis, idx in arm["joints"]:
            fill = JOINT_COLORS[idx] if idx == active else CARD
            add_dot(center, 7 if idx == active else 5, fill, TEXT, 2)
        add_dot(arm["tip"], 5, "#EF4444", CARD, 1)

        # 远 -> 近 绘制
        for _d, _seq, fn in sorted(items, key=lambda it: (-it[0], it[1])):
            fn()

        # 关节编号（不参与深度排序，始终置顶）
        for center, _axis, idx in arm["joints"]:
            s = P(center)
            c.create_text(s[0] + 12, s[1] - 11, text="J{}".format(idx + 1),
                          fill=JOINT_COLORS[idx], font=("Consolas", 9, "bold"), anchor="w")

        # 顶部六关节指示条：随画布宽度自适应，窄窗口只显示编号
        cell = W / 6.0
        for i in range(6):
            on = (i == active)
            c.create_rectangle(i * cell + 3, 6, (i + 1) * cell - 3, 24,
                               fill=JOINT_COLORS[i] if on else "#EEF2F7", outline="")
            txt = "J{} {:+.0f}".format(i + 1, a[i]) if cell >= 62 else "J{}".format(i + 1)
            c.create_text(i * cell + cell / 2, 15, text=txt,
                          fill="#FFFFFF" if on else JOINT_COLORS[i], font=FONT_TINY)

        state = "当前演示 J{}".format(active + 1) if active >= 0 else "六轴联动"
        hint = ("拖动旋转视角 | 滚轮缩放 | 双击复位    " + state) if W >= 470 else state
        c.create_text(10, H - 12, anchor="w", fill=MUTED, font=FONT_SMALL, text=hint)
        if W >= 430:
            c.create_text(W - 10, H - 12, anchor="e", fill=MUTED, font=FONT_TINY,
                          text="几何示意 / 非精确 DH 解算")

    def _on_close(self):
        self._stop()
        if self.p340_view is not None:
            self.p340_view.close()
        if self.reader is not None:
            self.reader.join(timeout=2.0)
        self.destroy()


# ---------------------------------------------------------------- 入口
def selftest():
    src = AngleSource("", 115200, demo=True)
    src.open()
    for preset in DEMO_PRESETS:
        src.preset = preset
        for t in (0.0, 1.3, 2.6, 5.1, 8.7, 16.2):
            vals, active = AngleSource.demo_angles(t, preset)
            assert len(vals) == 6, "preset {} sample count".format(preset)
            assert all(math.isfinite(v) for v in vals), "preset {} non-finite".format(preset)
            assert -1 <= active < 6, "preset {} active index".format(preset)
        print("PRESET_OK index={}".format(DEMO_PRESETS.index(preset)))
    src.preset = DEMO_PRESETS[0]
    for i in range(3):
        angles, coords, ms, active = src.read()
        assert angles is not None and len(angles) == 6, "demo 数据异常"
        print("SAMPLE {} angles={} read_ms={:.2f}".format(i + 1, angles, ms))
    src.close()

    arm = build_arm([12.5, -30.0, 65.0, -35.0, 40.0, 80.0])
    assert len(arm["segments"]) == 6, "连杆段数应为 6"
    assert len(arm["joints"]) == 6, "关节数应为 6"
    assert len(arm["fingers"]) == 2, "夹爪片数应为 2"
    R = arm["frame"]
    for i in range(3):
        assert abs(sum(R[i][k] ** 2 for k in range(3)) - 1.0) < 1e-9, "旋转矩阵行非单位"
    for i in range(3):
        for j in range(i + 1, 3):
            assert abs(sum(R[i][k] * R[j][k] for k in range(3))) < 1e-9, "旋转矩阵行不正交"
    for p0, p1, _ in arm["segments"]:
        for v in (p0, p1):
            assert all(math.isfinite(x) for x in v), "3D 点非有限值"
            s = project(v, 0.6, 0.4, POSE_SCALE, 225, 240)
            assert all(math.isfinite(x) for x in s), "投影非有限值"
    cur = [0.0] * 6
    for _ in range(80):
        cur, moving = lerp_angles(cur, [90.0] * 6)
    assert all(abs(v - 90.0) < 0.01 for v in cur), "插值未收敛"
    assert not moving, "收敛后不应继续移动"
    wrapped, _m = lerp_angles([179.0] * 6, [-179.0] * 6, factor=0.5)
    assert all(v > 179.0 or v < -179.0 for v in wrapped), "±180 跨界未走近路"
    print("LERP_OK converged=True wrap_shortest=True")

    # 关节轴向回归：腕部三轴两两垂直、肩肘共面、J5 横销落在 J5 关节点
    def axis_angle(u, v):
        c = sum(x * y for x, y in zip(vnorm(u), vnorm(v)))
        return math.degrees(math.acos(max(-1.0, min(1.0, abs(c)))))

    for ang in ([0.0] * 6,
                [12.5, -30.0, 65.0, -35.0, 40.0, 80.0],
                [-90.0, 60.0, -60.0, 45.0, -120.0, 170.0]):
        chk = build_arm(ang)
        ax = [j[1] for j in chk["joints"]]
        assert axis_angle(ax[1], ax[2]) < 5.0, "J2/J3 肩肘应共面"
        assert axis_angle(ax[2], ax[3]) > 80.0, "J4 不得与 J3 平行（腕俯仰画成第三个肘）"
        assert axis_angle(ax[3], ax[4]) > 80.0, "J4-J5 应垂直"
        assert axis_angle(ax[4], ax[5]) > 80.0, "J5-J6 不得同轴"
        mid = [(chk["cross_bar"][0][k] + chk["cross_bar"][1][k]) / 2.0 for k in range(3)]
        d5 = math.sqrt(sum((x - y) ** 2 for x, y in zip(mid, chk["joints"][4][0])))
        assert d5 < 1e-6, "J5 横销应位于 J5 关节点"
    print("AXIS_OK wrist_orthogonal=True cross_bar_on_J5=True")

    print("POSE3D_OK segments=6 joints=6 orthogonal=True projection=True")
    print("IMPORT_OK MyCobot280 / pyserial / tkinter")

    # 新增：320 M5 适配层自检（纯软件，不打开串口）
    p320 = profiles.get_profile("320")
    ok320, msg320 = profiles.driver_available(p320)
    assert ok320, "320 驱动不可用: {}".format(msg320)
    geo320 = profiles.scale_geometry(BASE_H, LINK_LEN, 96.0, p320.geom_scale)
    arm320 = build_arm([12.5, -30.0, 65.0, -35.0, 40.0, 80.0], geo320)
    assert abs(arm320["grid_extent"] - 120.0) < 1e-9, "320 工作半径比例应为 1.25"
    assert abs(arm320["base_h"] - BASE_H * 1.25) < 1e-9, "320 基座高度未按比例缩放"
    assert len(arm320["segments"]) == 6 and len(arm320["joints"]) == 6
    assert all(math.isfinite(x) for p0, p1, _ in arm320["segments"]
               for v in (p0, p1) for x in v), "320 3D 点非有限值"
    assert profiles.over_limit(p320, [0, 0, 0, 0, 0, 0]) == []
    assert profiles.over_limit(p320, [170, 0, 0, 0, 0, 0]) == [1]
    assert profiles.over_limit(profiles.get_profile(profiles.DEFAULT_MODEL),
                               [179, 0, 0, 0, 0, 0]) == []
    p340 = profiles.get_profile("p340")
    ok340, msg340 = profiles.driver_available(p340)
    assert ok340, "P340 驱动不可用: {}".format(msg340)
    assert p340.angle_api == "get_angles_info" and p340.coords_api == "get_coords_info"
    assert p340.dof == 3 and p340.dof_options == (3, 4)

    # 新增：四个演示预设在 3 / 4 轴下均合法、不越官方限位
    import p340_view
    assert tuple(p340_view.DEMO_PRESETS) == DEMO_PRESETS, "P340 预设与主动作下拉不一致"
    for preset in DEMO_PRESETS:
        for axes in (3, 4):
            lim = p340.joint_limits_alt if axes == 4 else p340.joint_limits
            for t in (0.0, 1.3, 2.6, 5.1, 8.7, 16.2):
                vals, act = p340_view.demo_angles(t, preset, axes)
                assert len(vals) == axes and all(math.isfinite(v) for v in vals)
                assert -1 <= act < axes
                assert not profiles.over_limit(p340, vals, lim), \
                    "P340 预设 {} {} 轴越界: {}".format(preset, axes, vals)
    print("P340_PRESET_OK presets={} axes=3/4 in_limits=True".format(len(DEMO_PRESETS)))

    assert profiles.over_limit(p340, [0, 100, 0]) == [2], "J2 超限未识别"
    assert profiles.over_limit(p340, [0, 0, 100]) == [3], "3 轴模式 J3 超限未识别"
    assert profiles.over_limit(p340, [0, 0, 100], p340.joint_limits_alt) == [], \
        "4 轴模式 J3 上限应为 110"
    print("P340_OK driver={} dof={} axes={} limits_ok=True".format(
        p340.driver_class, p340.dof, p340.dof_options))
    print("PROFILE_OK keys={} 320_driver={} geom_scale={} limits_ok=True".format(
        profiles.model_keys(), p320.driver_class, p320.geom_scale))
    print("SELFTEST_OK")


def pose_test():
    """创建真实 tkinter 窗口，遍历 3D 绘制与多尺寸自适应路径（自动化验证）。"""
    app = MonitorApp(force_demo=False)
    app.withdraw()
    app.update()
    samples = (
        [0.0] * 6,
        [30.0, -40.0, 70.0, -30.0, 45.0, 90.0],
        [-90.0, 60.0, -60.0, 45.0, -120.0, 170.0],
    )
    for i, ang in enumerate(samples):
        for active in (-1, i):
            app._draw_pose(ang, active)
            app.update()
        app.view_az += 0.5
        app.view_el = 0.9
        app._draw_pose(ang)
        app.update()
        app._draw_trend()
    app._pose_reset()
    app._pose_wheel(type("E", (), {"delta": 120})())
    app.update()
    print("TK_OK POSE_DRAW_OK draws={} canvas_items={}".format(len(samples) * 3, len(app.pose.find_all())))

    # 尺寸自适应：不同画布尺寸 / 姿态 / 视角下，所有 3D 点必须完整落在画布内
    sizes = ((450, 320), (360, 220), (700, 520), (240, 180), (1200, 800), (340, 600))
    cases = 0
    for W, H in sizes:
        for ang in samples:
            for az, el in ((0.0, 0.4), (0.9, 1.1), (2.4, 0.0)):
                arm = build_arm(ang)
                app._fit_snap = True
                sc, ox, oy = app._fit_pose(arm, W, H, az, el)
                for p in pose_points(arm):
                    sx, sy, _d = project(p, az, el, sc, ox, oy)
                    assert -2.0 <= sx <= W + 2.0 and -2.0 <= sy <= H + 2.0, \
                        "画布 {}x{} 内容越界 ({:.1f},{:.1f})".format(W, H, sx, sy)
                cases += 1
    print("FIT_OK cases={} sizes={} all_points_inside=True".format(cases, len(sizes)))

    # 滚动宿主：窗口过矮时启用纵向滚动条，够高时隐藏
    Evt = lambda w, h: type("E", (), {"width": w, "height": h})()
    app._on_host_configure(Evt(1100, 760))
    tall = app._vsb_visible
    app._on_host_configure(Evt(1100, 300))
    short = app._vsb_visible
    app._on_host_configure(Evt(1100, 760))
    restored = app._vsb_visible
    assert (not tall) and short and (not restored), "滚动条按需显示逻辑异常"
    print("SCROLL_OK tall={} short={} restored={}".format(tall, short, restored))

    # 顶层窗口真实 resize（若环境无法映射窗口则记录为 skip）
    try:
        app.deiconify()
        pose_heights = []
        clips = []
        bars = []
        areas = []
        aligned = []
        for W, H in ((1160, 900), (1000, 700), (880, 600), (1400, 1000), (880, 900)):
            app.geometry("{}x{}".format(W, H))
            app.update()
            app._after_resize()          # 手动触发防抖后的实测兜底
            app.update()
            pose_heights.append(app.pose.winfo_height())
            reqw, actualw = app.toolbar.winfo_reqwidth(), app.toolbar.winfo_width()
            assert reqw <= actualw + 2, "单行工具栏被挤压 req={} actual={}".format(reqw, actualw)
            assert app.run_btn.winfo_x() + app.run_btn.winfo_width() <= actualw + 2, "工具栏按钮溢出"
            bars.append((reqw, actualw))
            assert app.pose.winfo_width() > 40 and app.pose.winfo_height() > 40, "画布未获得尺寸"
            assert app.pose.winfo_height() >= 350, "3D 演示独立纵列高度不足"
            assert app.center_frame.winfo_width() > app.left_frame.winfo_width(), "3D 列不是主纵列"
            assert app.trend.winfo_width() > 40, "趋势画布未获得尺寸"
            areas.append(app.pose.winfo_width() * app.pose.winfo_height())
            body_bottom = app.body.winfo_rooty() + app.body.winfo_height()
            clip_f3 = app.f3.winfo_rooty() + app.f3.winfo_height() - body_bottom
            clip_tip = app.tip_label.winfo_rooty() + app.tip_label.winfo_height() - body_bottom
            assert clip_f3 <= 2 and clip_tip <= 2, \
                "内容被裁切 f3={:+.0f} tip={:+.0f}".format(clip_f3, clip_tip)
            clips.append((int(clip_f3), int(clip_tip)))
            # 非滚动场景下右列底边应与主区底边对齐（留白 <=40px）
            if app.body.winfo_height() <= app.host_canvas.winfo_height() + 2:
                assert clip_f3 >= -40, "右列未与主区底部对齐 gap={:.0f}px".format(-clip_f3)
                aligned.append(int(clip_f3))
            app._fit_snap = True
            app._draw_pose(samples[1])
            app._draw_trend()
            app.update()
            print("RESIZE_OK {}x{} pose={}x{} trend={}x{} host={} body={} right={} reqh={} cols={}/{}/{}".format(
                W, H, app.pose.winfo_width(), app.pose.winfo_height(),
                app.trend.winfo_width(), app.trend.winfo_height(),
                app.host_canvas.winfo_height(), app.body.winfo_height(),
                app.right_frame.winfo_height(), app.body.winfo_reqheight(),
                app.left_frame.winfo_width(), app.center_frame.winfo_width(),
                app.right_frame.winfo_width()))
        assert len(set(pose_heights)) > 1, "画布高度未随窗口高度变化"
        print("VERTICAL_OK distinct_heights={}".format(sorted(set(pose_heights))))
        print("CLIP_OK no_overflow cases={} worst_f3={} worst_tip={}".format(
            len(clips), max(c[0] for c in clips), max(c[1] for c in clips)))
        print("TOOLBAR_OK single_row cases={} min_slack={}".format(
            len(bars), min(a - r for r, a in bars)))
        print("POSE_COLUMN_OK min_height={} max_area={}px2".format(
            min(pose_heights), max(areas)))
        print("ALIGN_OK cases={} worst_gap={}px".format(
            len(aligned), -min(aligned) if aligned else -1))
    except Exception as exc:
        print("RESIZE_GUI_SKIPPED reason={} msg={}".format(type(exc).__name__, exc))
    finally:
        app.withdraw()

    app._stop()
    app.destroy()


def main():
    ap = argparse.ArgumentParser(
        description="myCobot 280 / 320 M5 与 ultraArm P340 关节实时监控")
    ap.add_argument("--model", default=profiles.DEFAULT_MODEL, choices=profiles.model_keys(),
                    help="机械臂型号：280 默认 / 320 六轴 / p340 四轴码垛臂")
    ap.add_argument("--demo", action="store_true", help="启动即进入演示模式并自动开始动画")
    ap.add_argument("--selftest", action="store_true", help="无界面自检并退出")
    ap.add_argument("--pose-test", action="store_true", help="创建窗口验证 3D 绘制后退出")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return 0
    if args.pose_test:
        pose_test()
        return 0
    app = MonitorApp(force_demo=args.demo, model=args.model)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
