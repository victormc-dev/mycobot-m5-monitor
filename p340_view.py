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
import p340_motion as motion_mod
from p340_motion import MotionRejected

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
POSE_CHROME = 210           # 中间卡除画布外的固定高度（标题+模式行+脚注+边距）
TREND_CHROME = 232          # 右侧趋势卡除画布外的固定高度

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
    """P340 真机 / 演示数据源。

    真机侧遵守步进电机的三条硬约束（见 p340_motion 模块说明）：
      * 回零前不读关节角——读出来也不可信，角度改由 P340Motion 软件模型提供；
      * 所有运动指令走命令队列，在本采集线程里串行执行，避免多线程抢串口；
      * 回零用自带超时的 g28()，不用驱动里会死等的 go_zero()。
    """

    def __init__(self, profile, port, baud, demo, axis_mode_getter,
                 preset_getter=None, motion=None):
        self.profile = profile
        self.port = port
        self.baud = baud
        self.demo = demo
        self.axis_mode_getter = axis_mode_getter
        self.preset_getter = preset_getter or (lambda: DEMO_PRESETS[0])
        self.motion = motion or motion_mod.P340Motion(
            axes=profile.dof, limits_3=profile.joint_limits,
            limits_4=profile.joint_limits_alt)
        self.robot = None
        self.cmds = queue.Queue()
        self.report = lambda _text: None      # 由面板注入，回传状态到 UI 线程
        self._n = 0
        self._coords = None

    # ---- 生命周期
    def open(self):
        if self.demo:
            return
        if not self.port:
            raise ValueError("未选择串口，请先点击「刷新」扫描端口")
        self.robot = profiles.create_robot(self.profile, self.port, self.baud)
        try:
            self.robot.set_speed_mode(0)      # 匀速模式，拖动单步指令更可预期
        except Exception:
            pass
        self.motion.reset("已连接，等待回零")

    def close(self):
        try:
            if self.robot is not None:
                self.robot._serial_port.close()
        except Exception:
            pass
        self.robot = None

    # ---- 指令队列（UI 线程只入队，串口操作全部落在这里）
    def submit(self, kind, payload=None):
        self.cmds.put((kind, payload))

    def _drain(self):
        while True:
            try:
                kind, payload = self.cmds.get_nowait()
            except queue.Empty:
                return
            try:
                self._exec(kind, payload)
            except Exception as exc:
                self.report("指令 {} 异常：{}: {}".format(
                    kind, type(exc).__name__, exc))

    def _exec(self, kind, payload):
        if self.demo:
            self._exec_demo(kind, payload)
            return
        if self.robot is None:
            self.report("串口未连接，指令已丢弃")
            return
        if kind == "home":
            self._do_home()
        elif kind == "send":
            self._do_send(*payload)
        elif kind == "mode":
            self._do_mode(payload)
        elif kind == "release":
            self.robot.release_all_servos()
            self.motion.reset("已松轴：位置未知，需重新回零")
            self.report("全部关节已掉电（可手动示教）；继续运动前必须重新回零")
        elif kind == "power":
            self.robot.power_on()
            self.report("全部关节已上电")

    def _exec_demo(self, kind, payload):
        """演示模式下这些指令只做本地状态迁移，不涉及硬件。"""
        if kind == "home":
            self.motion.mark_homed()
            self.report("演示模式：已模拟回零，角度归零")
        elif kind == "release":
            self.motion.reset("演示模式：已模拟松轴")
            self.report("演示模式：已模拟松轴（需重新回零）")

    def _do_home(self):
        self.report("回零中（G28），约 10-20 秒，请勿触碰机械臂…")
        ok = motion_mod.g28(self.robot, motion_mod.HOME_TIMEOUT)
        if not ok:
            self.motion.reset("回零失败")
            self.report("回零失败：未收到 ok 应答（检查 12V 供电 / 串口 / 限位开关）")
            return
        try:
            self.robot.set_mode(1 if self.motion.mode == motion_mod.MODE_REL else 0)
        except Exception:
            pass
        sw = motion_mod.switch_state(self.robot)
        self.motion.mark_homed()
        self.report("回零完成，已切{}模式{}".format(
            motion_mod.MODE_LABELS[self.motion.mode],
            "；限位开关 {}".format(sw) if sw else ""))

    def _do_mode(self, mode):
        self.motion.set_mode(mode)
        try:
            self.robot.set_mode(1 if mode == motion_mod.MODE_REL else 0)
        except Exception as exc:
            self.report("切换运动模式失败：{}".format(exc))
            return
        self.report("运动模式：{}".format(motion_mod.MODE_LABELS[mode]))

    def _do_send(self, axis, target, delta):
        if not self.motion.homed:
            self.report("未回零：指令已拦截（先点「回零 G28」）")
            return
        speed = int(self.motion.speed)
        try:
            if self.motion.mode == motion_mod.MODE_REL:
                self.robot.set_mode(1)                     # 每条指令前重申模态
                self.robot.set_angle(axis + 1, round(delta, 2), speed)
            else:
                self.robot.set_mode(0)
                self.robot.set_angle(axis + 1, round(target, 2), speed)
        except Exception as exc:
            self.report("J{} 下发失败：{}: {}".format(axis + 1, type(exc).__name__, exc))
            return
        self.motion.commit(axis, target)
        self.report("J{} {} {:+.2f}° → 目标 {:+.2f}°".format(
            axis + 1, "相对" if self.motion.mode == motion_mod.MODE_REL else "绝对",
            delta if self.motion.mode == motion_mod.MODE_REL else target,
            self.motion.joints[axis]))

    # ---- 采样
    def read(self):
        t0 = time.perf_counter()
        n = self.axis_mode_getter()
        if self.demo:
            angles, active = demo_angles(time.monotonic(), self.preset_getter(), n)
            ms = (time.perf_counter() - t0) * 1000.0
            return angles, None, ms, active

        self._n += 1
        self._drain()
        # 关节角不读串口，直接取软件模型；坐标仅在回零后低频读取（带超时）
        if self.motion.homed and self._n % 5 == 1:
            coords = motion_mod.read_coords(self.robot, timeout=0.6)
            if coords:
                self._coords = coords
        ms = (time.perf_counter() - t0) * 1000.0
        return self.motion.snapshot(n), self._coords, ms, -1



class P340Panel(tk.Frame):
    """四轴监控面板。对外接口：start / stop / close / set_axis_mode。"""

    def __init__(self, parent, on_status=None):
        super().__init__(parent, bg=BG)
        self.profile = profiles.get_profile("p340")
        self.on_status = on_status or (lambda _text: None)
        self.axis_var = tk.IntVar(value=self.profile.dof)   # 3 / 4 轴
        self._axis_n = self.profile.dof                    # 采集线程读取，避免跨线程访问 tk 变量
        self._preset = DEMO_PRESETS[0]                     # 采集线程读取当前演示预设
        # 运动门禁：步进电机必须回零后才能下发，角度来自软件模型而非串口读回
        self.motion = motion_mod.P340Motion(
            axes=self.profile.dof, limits_3=self.profile.joint_limits,
            limits_4=self.profile.joint_limits_alt)
        self.source = None                # 采集线程内的数据源，用于入队指令
        self.mode_var = tk.StringVar(value=motion_mod.MODE_REL)
        self.speed_var = tk.IntVar(value=motion_mod.DEFAULT_SPEED)
        # 角度条拖动状态
        self._bar_dragging = -1
        self._bar_edit = [0.0] * self.profile.max_joints
        self._hint_shown = True          # 矮窗口下隐藏底部操作提示，避免被裁切
        self._dense = False              # 矮窗口下压缩关节行间距
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
        top = tk.Frame(head, bg=CARD)
        top.pack(fill="x")
        tk.Label(top, text="ultraArm P340 · 四轴码垛步进机械臂监控", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(side="left", padx=10, pady=(8, 2))
        self.spec_label = tk.Label(top, text=profiles.summary(self.profile), bg=CARD,
                                   fg=MUTED, font=FONT_SMALL)
        self.spec_label.pack(side="left", padx=4, pady=(8, 2))
        self.home_badge = tk.Label(top, text="● 未回零", bg="#FEF2F2", fg="#B91C1C",
                                   font=FONT_UI_B, padx=8, pady=2)
        self.home_badge.pack(side="right", padx=10, pady=(8, 2))

        # 第二行：回零门禁 + 运动参数。步进电机必须先 G28 才能发运动指令
        ctl = tk.Frame(head, bg=CARD)
        ctl.pack(fill="x", padx=10, pady=(0, 8))
        tk.Label(ctl, text="自由度", bg=CARD, fg=MUTED, font=FONT_SMALL).pack(
            side="left", padx=(0, 2))
        for n in self.profile.dof_options:
            tk.Radiobutton(ctl, text="{} 轴".format(n), value=n, variable=self.axis_var,
                           indicatoron=False, bg="#EDF1F6", selectcolor="#DBEAFE",
                           fg=TEXT, font=FONT_SMALL, relief="flat", padx=8, pady=1,
                           command=self.set_axis_mode).pack(side="left", padx=2)
        self.home_btn = tk.Button(ctl, text="回零 G28", command=self._request_home,
                                  font=FONT_UI_B, bg="#2563EB", fg="white",
                                  relief="flat", padx=10, pady=1)
        self.home_btn.pack(side="left", padx=(12, 2))
        tk.Button(ctl, text="松轴示教", command=lambda: self._submit("release"),
                  font=FONT_SMALL, bg="#EDF1F6", fg=TEXT, relief="flat",
                  padx=8, pady=1).pack(side="left", padx=2)
        tk.Button(ctl, text="上电", command=lambda: self._submit("power"),
                  font=FONT_SMALL, bg="#EDF1F6", fg=TEXT, relief="flat",
                  padx=8, pady=1).pack(side="left", padx=2)
        tk.Label(ctl, text="运动", bg=CARD, fg=MUTED, font=FONT_SMALL).pack(
            side="left", padx=(12, 2))
        for label, val in ((motion_mod.MODE_LABELS[motion_mod.MODE_REL], motion_mod.MODE_REL),
                           (motion_mod.MODE_LABELS[motion_mod.MODE_ABS], motion_mod.MODE_ABS)):
            tk.Radiobutton(ctl, text=label, value=val, variable=self.mode_var,
                           indicatoron=False, bg="#EDF1F6", selectcolor="#DBEAFE",
                           fg=TEXT, font=FONT_SMALL, relief="flat", padx=6, pady=1,
                           command=self._on_mode_change).pack(side="left", padx=2)
        tk.Label(ctl, text="速度", bg=CARD, fg=MUTED, font=FONT_SMALL).pack(
            side="left", padx=(10, 2))
        tk.Spinbox(ctl, from_=10, to=200, increment=10, width=4, font=FONT_SMALL,
                   textvariable=self.speed_var, relief="flat", bg="#F3F5F8",
                   command=self._on_speed_change).pack(side="left")

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
            # 拖动角度条：按下/拖动只做预览，松开才下发一条相对增量指令
            c.bind("<ButtonPress-1>", lambda e, idx=i: self._bar_press(e, idx))
            c.bind("<B1-Motion>", lambda e, idx=i: self._bar_drag(e, idx))
            c.bind("<ButtonRelease-1>", lambda e, idx=i: self._bar_release(e, idx))
            c.configure(cursor="sb_h_double_arrow")
            self.bar_canvases.append(c)
        tk.Label(left, text="12V 5A · 115200 波特率 · 步进电机\n"
                            "回零后角度为软件模型值（不读串口）",
                 bg=CARD, fg=MUTED, font=FONT_SMALL, justify="left").pack(
            anchor="w", padx=12, pady=(10, 4))
        self.op_hint = tk.Label(left, text="拖动角度条 → 松手下发相对增量（≤±45°/次）",
                                bg=CARD, fg=MUTED, font=FONT_TINY, justify="left",
                                wraplength=230)
        self.op_hint.pack(anchor="w", padx=12, pady=(0, 10))

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

    def fit_height(self, avail):
        """按宿主可用高度重排两个画布。

        P340 面板是画布宿主里的 window item，若只靠自然高度，画布会按内容请求
        尺寸不断长大、把纵向滚动条顶出来（1226 级“棘轮”）。这里显式给定高度：
        中间 3D 卡与右侧趋势卡各自铺满，卡片其余部分的高度由 chrome 常量扣除。
        """
        avail = max(360, int(avail))
        self.pose.configure(height=max(200, avail - POSE_CHROME))
        self.trend.configure(height=max(140, avail - TREND_CHROME))
        want_hint = avail >= 560
        if want_hint != self._hint_shown:
            if want_hint:
                self.op_hint.pack(anchor="w", padx=12, pady=(0, 10))
            else:
                self.op_hint.pack_forget()
            self._hint_shown = want_hint
        dense = avail < 620
        if dense != self._dense:
            for row in self.row_frames:
                row.pack_configure(pady=3 if dense else 5)
            self._dense = dense

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

    # ---------------------------------------------------------------- 指令下发
    def _submit(self, kind, payload=None):
        """把指令塞进采集线程的队列；串口只由那一个线程操作。"""
        if self.demo:
            # 演示模式无需串口，直接在本线程模拟状态迁移
            if kind == "home":
                self.motion.mark_homed()
                self.on_status("演示模式：已模拟回零，角度归零")
            elif kind == "release":
                self.motion.reset("演示模式：已模拟松轴")
                self.on_status("演示模式：已模拟松轴（需重新回零）")
            self._refresh_home_badge()
            return
        if not self.running or self.source is None:
            self.on_status("P340 未连接：请先在主工具栏点「开始采集」建立串口")
            return
        self.source.submit(kind, payload)

    def _request_home(self):
        """回零入口。步进电机每次上电都必须先 G28，运动指令才被放行。"""
        if self.running and not self.demo:
            self.home_btn.config(state="disabled", text="回零中…")
            self.on_status("P340 发送 G28 回零，请等待约 10-20 秒…")
        self._submit("home")

    def _on_mode_change(self):
        mode = self.mode_var.get()
        self.motion.set_mode(mode)
        if self.demo or not self.running:
            self.on_status("运动模式：{}".format(motion_mod.MODE_LABELS[mode]))
            return
        self._submit("mode", mode)

    def _on_speed_change(self):
        try:
            self.motion.speed = max(10, min(200, int(self.speed_var.get())))
        except (tk.TclError, ValueError):
            self.speed_var.set(self.motion.speed)
            return
        self.on_status("运动速度：{} mm/s".format(self.motion.speed))

    def _refresh_home_badge(self):
        """回零状态徽标：未回零 / 回零中 / 已回零。"""
        if self.demo:
            self.home_badge.config(text="● 演示·已模拟回零", bg="#EFF6FF", fg="#1D4ED8")
            self.home_btn.config(state="normal", text="模拟回零")
            return
        if self.motion.homed:
            self.home_badge.config(text="● 已回零", bg="#ECFDF5", fg="#047857")
            self.home_btn.config(state="normal", text="重新回零 G28")
            return
        if self.running and str(self.home_btn["state"]) == "disabled":
            self.home_badge.config(text="● 回零中", bg="#FEF9C3", fg="#A16207")
            return
        self.home_badge.config(text="● 未回零", bg="#FEF2F2", fg="#B91C1C")
        if str(self.home_btn["state"]) == "disabled":
            self.home_btn.config(state="normal", text="回零 G28")

    # ---------------------------------------------------------------- 角度条拖动
    def _can_drag(self):
        """演示模式随时可拖；真机必须先回零，否则角度不可信、指令也会被拦截。"""
        if self.demo:
            return True
        return bool(self.running and self.motion.homed)

    def _x_to_angle(self, i, x):
        """角度条 x 坐标 → 角度，与 _draw_bar 的 0 线居中映射互逆。"""
        c = self.bar_canvases[i]
        w = c.winfo_width()
        if w < 20:                       # 未完成布局时退回请求宽度，与 _draw_bar 一致
            w = c.winfo_reqwidth()
        w = max(120, w)
        pad = 3
        mid = w / 2.0
        left_w = max(1.0, mid - pad)
        right_w = max(1.0, w - pad - mid)
        lo, hi = self._limit_at(i)
        neg_span = max(1e-6, -lo) if lo < 0 else 1e-6
        pos_span = max(1e-6, hi) if hi > 0 else 1e-6
        if x >= mid:
            val = (x - mid) / right_w * pos_span
        else:
            val = -(mid - x) / left_w * neg_span
        return max(float(lo), min(float(hi), val))

    def _paint_row(self, i, val):
        lo, hi = self._limit_at(i)
        self._draw_bar(self.bar_canvases[i], val, lo, hi, JOINT_COLORS[i])
        self.value_labels[i].configure(
            text="{:+.2f}".format(val) if val is not None else "--.--")

    def _bar_press(self, evt, idx):
        if not self._can_drag():
            self.on_status("P340 未回零：先点「回零 G28」再拖动角度条")
            return
        self._bar_dragging = idx
        self._bar_edit[idx] = self._x_to_angle(idx, evt.x)
        self._paint_row(idx, self._bar_edit[idx])
        self.last_angles[idx] = self._bar_edit[idx]
        self._draw_pose(self.last_angles)

    def _bar_drag(self, evt, idx):
        if self._bar_dragging != idx:
            return
        self._bar_edit[idx] = self._x_to_angle(idx, evt.x)
        self._paint_row(idx, self._bar_edit[idx])
        self.last_angles[idx] = self._bar_edit[idx]
        self._draw_pose(self.last_angles)

    def _bar_release(self, evt, idx):
        if self._bar_dragging != idx:
            return
        self._bar_dragging = -1
        target = self._x_to_angle(idx, evt.x)
        self._bar_edit[idx] = target

        if self.demo:
            self.last_angles[idx] = target
            self._paint_row(idx, target)
            self._draw_pose(self.last_angles)
            self.on_status("P340 演示：J{} 预览 {:+.2f}°（未下发）".format(idx + 1, target))
            return

        try:
            tgt, delta = self.motion.plan(idx, target)
        except MotionRejected as exc:
            base = self.motion.joints[idx]
            self.last_angles[idx] = base
            self._paint_row(idx, base)
            self._draw_pose(self.last_angles)
            self.on_status("P340 指令未下发：{}".format(exc))
            return

        # 乐观显示：先按目标角画，真正结果由采集线程回报
        self.last_angles[idx] = tgt
        self._paint_row(idx, tgt)
        self._draw_pose(self.last_angles)
        self._submit("send", (idx, tgt, delta))
        self.on_status("P340 下发 J{} {} {:+.2f}°（目标 {:+.2f}°）".format(
            idx + 1,
            "相对" if self.motion.mode == motion_mod.MODE_REL else "绝对",
            delta if self.motion.mode == motion_mod.MODE_REL else tgt, tgt))

    def set_axis_mode(self):
        """3 轴 / 4 轴切换：官方标注自由度 3-4，第 4 轴为选配。"""
        self._axis_n = self.axis_var.get()
        self.motion.set_axes(self._axis_n)
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
        self._bar_dragging = -1
        self.source = None
        # 每次建立连接都回到未回零状态：步进电机掉电后位置未知，必须重新 G28
        if demo:
            self.motion.mark_homed()
        else:
            self.motion.reset("等待回零")
        self._refresh_home_badge()
        self.stop_evt = threading.Event()
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        self.on_status("P340 {}· {} 采集中...{}".format(
            "演示模式" if demo else "真机", self._preset,
            "" if demo else "（请先回零 G28）"))

    def _reader(self):
        src = _P340Source(self.profile, self.port, self.baud, self.demo,
                          lambda: self._axis_n, lambda: self._preset,
                          motion=self.motion)
        src.report = lambda text: self.q.put(("status", text))
        self.source = src
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
            self.source = None
            self.q.put(("closed", None))

    def stop(self):
        self.stop_evt.set()
        self.running = False
        self._bar_dragging = -1
        self.motion.reset("采集已停止")
        self._refresh_home_badge()
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
                    self.motion.reset("连接异常")
                    self._refresh_home_badge()
                elif kind == "status":
                    self.on_status(str(payload))
                    self._refresh_home_badge()
                elif kind == "ready":
                    self._refresh_home_badge()
                elif kind == "data":
                    self._update(*payload)
                elif kind == "closed":
                    self.running = False
                    self.motion.reset("连接已断开")
                    self._refresh_home_badge()
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
        # 真机未回零时关节角没有参考基准，全部显示 "--" 而不是假的 0.00
        live = self.demo or self.motion.homed
        if live:
            self.history.append(vals)
            if len(self.history) > HISTORY:
                self.history.pop(0)
        for i in range(self.profile.max_joints):
            if i == self._bar_dragging:
                continue                     # 拖动预览优先，不被采样帧覆盖
            v = vals[i] if (i < len(vals) and live) else None
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
        if self.demo:
            tail = self._preset
        elif not self.motion.homed:
            tail = "未回零 · 角度不可用（点「回零 G28」）"
        else:
            tail = "{} 模式 · 角度=软件模型".format(
                motion_mod.MODE_LABELS[self.motion.mode])
        self.on_status("P340 | {} | {} | 帧 {} | 读取 {:.1f} ms{}{}".format(
            mode, tail, self.frames, elapsed_ms,
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


class _FakeSource:
    """UI 自检用的假数据源，只记录入队指令。"""

    def __init__(self):
        self.cmds = []

    def submit(self, kind, payload=None):
        self.cmds.append((kind, payload))


class _Evt:
    def __init__(self, x, y=9):
        self.x = x
        self.y = y


def panel_selftest():
    """创建真实 Tk 窗口，验证回零门禁与拖动发指令的完整链路（自动化）。"""
    root = tk.Tk()
    root.withdraw()
    panel = P340Panel(root)
    panel.pack(fill="both", expand=True)
    root.update()
    try:
        canvas = panel.bar_canvases[0]
        w = canvas.winfo_width()
        if w < 20:
            w = canvas.winfo_reqwidth()

        # 1) 演示模式：拖动只改预览，不入队
        panel.demo, panel.running = True, True
        panel.motion.mark_homed()
        panel._refresh_home_badge()
        before = panel.last_angles[0]
        panel._bar_press(_Evt(w * 0.85), 0)
        panel._bar_drag(_Evt(w * 0.90), 0)
        panel._bar_release(_Evt(w * 0.90), 0)
        assert panel.last_angles[0] != before, "演示拖动未更新预览"
        assert panel.source is None, "演示模式不应产生串口数据源"

        # 2) 真机未回零：拖动被拦截，角度保持不变
        panel.demo = False
        panel.motion.reset("自检：未回零")
        panel._refresh_home_badge()
        panel.source = _FakeSource()
        keep = panel.last_angles[0]
        panel._bar_press(_Evt(w * 0.10), 0)
        panel._bar_release(_Evt(w * 0.10), 0)
        assert panel.last_angles[0] == keep, "未回零仍然改动了角度"
        assert panel.source.cmds == [], "未回零仍然下发了指令"
        assert "未回零" in panel.home_badge["text"]

        # 3) 回零按钮入队
        panel._request_home()
        assert panel.source.cmds and panel.source.cmds[0][0] == "home", "回零未入队"

        # 4) 回零后拖动：下发一条相对增量，且不超过 MAX_STEP_DEG
        panel.motion.mark_homed()
        panel._refresh_home_badge()
        assert "已回零" in panel.home_badge["text"]
        panel.source.cmds.clear()
        panel._bar_press(_Evt(w * 0.95), 0)
        panel._bar_release(_Evt(w * 0.95), 0)
        kinds = [c[0] for c in panel.source.cmds]
        assert kinds == ["send"], "回零后拖动未下发指令: {}".format(kinds)
        _kind, (axis, tgt, delta) = panel.source.cmds[0]
        assert axis == 0 and 0 < delta <= motion_mod.MAX_STEP_DEG + 1e-6, \
            "增量越界: axis={} tgt={} delta={}".format(axis, tgt, delta)
        assert panel.motion.joints[0] == 0.0, "指令尚未执行，模型不应提前变化"

        # 5) 4 轴模式下行号校验
        panel.axis_var.set(4)
        panel.set_axis_mode()
        panel.motion.mark_homed()
        panel.source.cmds.clear()
        c4 = panel.bar_canvases[3]
        w4 = c4.winfo_width()
        if w4 < 20:
            w4 = c4.winfo_reqwidth()
        panel._bar_press(_Evt(w4 * 0.80), 3)
        panel._bar_release(_Evt(w4 * 0.80), 3)
        assert panel.source.cmds and panel.source.cmds[0][1][0] == 3, "J4 指令轴号错误"

        # 6) 真机链路：用假 robot 走一遍 G28 → 相对指令 → 未回零拦截
        from p340_motion import _FakeRobot
        panel.demo = False
        src = _P340Source(panel.profile, "COM_TEST", 115200, False, lambda: 3,
                          motion=panel.motion)
        src.robot = _FakeRobot(True)
        msgs = []
        src.report = msgs.append
        panel.motion.reset("测试")
        src._exec("send", (0, 20.0, 20.0))
        assert not any(c[0] == "set_angle" for c in src.robot.calls), \
            "未回零仍下发了关节指令"
        src.submit("home", None)
        src._drain()
        assert panel.motion.homed, "G28 应答后未置为已回零"
        src._exec("send", (0, 20.0, 20.0))
        assert panel.motion.joints[0] == 20.0, "相对指令未更新软件模型"
        assert ("set_angle", 1, 20.0, panel.motion.speed) in src.robot.calls, \
            "下行指令不是相对增量: {}".format(src.robot.calls)
        assert any("回零完成" in m for m in msgs), "回零完成未回报状态"
    finally:
        root.destroy()
    return True


__all__ = ["P340Panel", "DEMO_PRESETS", "demo_angles", "panel_selftest",
           "pose3d_selftest"]
