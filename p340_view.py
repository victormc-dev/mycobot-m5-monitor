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

        # 中：平面姿态示意
        mid = tk.Frame(body, bg=CARD, highlightthickness=1, highlightbackground=GRID)
        mid.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        tk.Label(mid, text="平面姿态示意（J1 基座 / J2 大臂 / J3 小臂）", bg=CARD, fg=TEXT,
                 font=FONT_UI_B).pack(anchor="w", padx=12, pady=(12, 2))
        self.pose = tk.Canvas(mid, width=330, height=360, bg=CARD, highlightthickness=0)
        self.pose.pack(fill="both", expand=True, padx=12, pady=(0, 4))
        tk.Label(mid, text="几何示意，非精确运动学解算；J4 选配轴以末端横销表示",
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

    def _draw_pose(self, angles):
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


__all__ = ["P340Panel", "DEMO_PRESETS", "demo_angles"]
