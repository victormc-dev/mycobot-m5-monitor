# coding=utf-8
"""ultraArm P340 运动控制层（新增模块）。

P340 是**步进电机 + 开环**结构，官方文档在每一个示例里都写了同一句话：

    「ultraArmP340 进行坐标运动/角度运动之前必须进行回零，否则无法获取到
      正确的角度/坐标。」

由此得到三条硬约束，本模块就是把它们落成代码：

1. **回零门禁**：上电后 `go_zero()`（G28）之前，关节角/坐标都不可信，任何运动
   指令都不许下发。
2. **相对坐标**：回零后进入 `set_mode(1)`（G91 相对笛卡尔）模式，拖动角度条下发
   的是**增量**，不是绝对角。
3. **不读关节角**：步进电机没有编码器，`get_angles_info()` 在未回零时不可用；
   因此界面上的关节角一律来自本模块维护的**软件模型**（回零清零 + 指令累加），
   不再依赖串口读回。

设计上刻意做成纯逻辑类（不 import tkinter、不碰串口），便于 `--selftest` 用假
robot 对象验证门禁、限位裁剪与增量计算。

    from p340_motion import P340Motion, MotionRejected, g28

    m = P340Motion(axes=3, limits_3=(-150, 170), ...)
    m.mark_homed()                       # G28 成功后调用
    target, delta = m.plan(0, 30.0)      # 目标角 -> (裁剪后的目标, 实际增量)
    robot.set_angle(1, delta, 50)        # 相对下发
    m.commit(0, target)                  # 更新软件模型
"""

from __future__ import annotations

import math
import time

HOME_TIMEOUT = 30.0      # G28 等待上限（秒）；超时即判定回零失败
MAX_STEP_DEG = 45.0      # 单次拖动允许的最大关节增量，防止误拖造成大幅运动
MIN_STEP_DEG = 0.05      # 小于该增量视为无操作，避免刷屏与空指令
DEFAULT_SPEED = 50       # 官方速度范围 0~200 (mm/s)

MODE_REL = "rel"         # G91 相对坐标（默认）
MODE_ABS = "abs"         # G90 绝对坐标（备用：个别固件把 M10 当绝对角时切这里）
MODE_LABELS = {MODE_REL: "相对 G91", MODE_ABS: "绝对 G90"}


class MotionRejected(Exception):
    """指令被门禁拒绝：未回零 / 超限 / 增量过小 / 轴号非法。"""


class P340Motion:
    """P340 关节状态门禁与相对指令规划。线程安全性由调用方（单采集线程）保证。"""

    def __init__(self, axes=3, limits_3=None, limits_4=None):
        self.axes = int(axes)
        self.limits_3 = tuple(limits_3) if limits_3 else ()
        self.limits_4 = tuple(limits_4) if limits_4 else ()
        self.mode = MODE_REL
        self.speed = DEFAULT_SPEED
        self.homed = False
        self.joints = [0.0, 0.0, 0.0, 0.0]   # 软件模型，仅前 axes 位有效
        self.sent = 0                         # 已成功下发的指令条数
        self.last_msg = "未回零"

    # ---- 限位
    def limits(self):
        if self.axes == 4 and self.limits_4:
            return self.limits_4
        return self.limits_3 or ((-180.0, 180.0),) * 4

    def limit_at(self, i):
        lim = self.limits()
        return lim[i] if i < len(lim) else (-180.0, 180.0)

    def clamp(self, i, value):
        lo, hi = self.limit_at(i)
        return max(lo, min(hi, float(value)))

    # ---- 状态迁移
    def set_axes(self, axes):
        """3 轴 / 4 轴切换。已回零状态保持（零点由限位开关确定，与轴数无关）。"""
        self.axes = int(axes)
        for i in range(self.axes, 4):
            self.joints[i] = 0.0

    def mark_homed(self):
        self.homed = True
        self.joints = [0.0, 0.0, 0.0, 0.0]
        self.sent = 0
        self.last_msg = "回零完成，角度已归零"

    def reset(self, why="未回零"):
        """断开连接 / 重新上电 / 停止采集后必须回到未回零状态。"""
        self.homed = False
        self.joints = [0.0, 0.0, 0.0, 0.0]
        self.last_msg = why

    def set_mode(self, mode):
        if mode not in (MODE_REL, MODE_ABS):
            raise ValueError("未知运动模式 {!r}".format(mode))
        self.mode = mode

    # ---- 指令规划
    def plan(self, i, target):
        """校验一次拖动目标角，返回 (裁剪后目标, 实际增量)。

        拒绝条件：未回零、轴号越界、增量过小。增量超限时按 MAX_STEP_DEG 截断，
        而不是直接报错——拖动条一次拉满很常见，截断更符合直觉。
        """
        if not self.homed:
            raise MotionRejected("未回零：请先点「回零 G28」")
        if not 0 <= i < self.axes:
            raise MotionRejected("J{} 不在当前 {} 轴模式内".format(i + 1, self.axes))
        tgt = self.clamp(i, target)
        delta = tgt - self.joints[i]
        if abs(delta) > MAX_STEP_DEG:
            tgt = self.clamp(i, self.joints[i] + math.copysign(MAX_STEP_DEG, delta))
            delta = tgt - self.joints[i]
        if abs(delta) < MIN_STEP_DEG:
            raise MotionRejected("J{} 增量仅 {:.2f}°，已忽略".format(i + 1, delta))
        return tgt, delta

    def commit(self, i, target):
        """指令下发成功后更新软件模型。"""
        self.joints[i] = self.clamp(i, target)
        self.sent += 1
        self.last_msg = "J{} -> {:+.2f}°".format(i + 1, self.joints[i])

    def snapshot(self, n=None):
        n = self.axes if n is None else int(n)
        return [float(v) for v in self.joints[:n]]


# ---------------------------------------------------------------- G28
def g28(robot, timeout=HOME_TIMEOUT, expect="ok"):
    """发送 G28 回零并等待应答，返回是否成功。

    不使用驱动自带的 `go_zero()`：它内部是 `while True` 死等 "ok"，机器人不应答
    （未上电 / 串口半开 / 限位开关故障）就会把采集线程永久卡住。这里加超时。
    """
    ser = getattr(robot, "_serial_port", None)
    if ser is None:                      # 非标准驱动：退回官方方法
        robot.go_zero()
        return True
    try:
        ser.reset_input_buffer()         # 丢掉上一条指令残留的 ok，避免误判成功
    except Exception:
        pass
    ser.write("G28\r")
    ser.flush()
    buf = ""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            pending = ser.inWaiting()
            if pending:
                buf += ser.read(pending).decode("utf-8", "ignore")
                if expect in buf.lower():
                    return True
        except Exception:
            return False
        time.sleep(0.05)
    return False


def switch_state(robot, timeout=0.8):
    """读取回零限位开关状态（M119）；无应答返回 None。用于回零后的复核提示。"""
    text = _query(robot, "M119", "ENDSTOP", timeout)
    if not text:
        return None
    seg = text[text.find("ENDSTOP"):].split("\n")[0]
    return seg.replace("ENDSTOP", "").strip(" :") or None


def read_coords(robot, timeout=0.8):
    """读取末端坐标（M114），带超时；无应答返回 None。

    不能直接用 profiles.read_coords()：驱动 `_request()` 在没有数据时是
    `while count < 5` 的忙等（count 只在解析失败时自增），波特率不匹配或固件
    不认 M114 会把采集线程永久卡住。这里改成带 deadline 的版本。
    """
    return _parse_floats(_query(robot, "M114", "COORDS", timeout))


def read_angles(robot, timeout=0.8):
    """读取关节角（M12），带超时。仅用于人工校验软件模型，不参与轮询。"""
    return _parse_floats(_query(robot, "M12", "ANGLES", timeout))


def _query(robot, command, token, timeout):
    """发一条 G-code 查询并按 deadline 收应答文本；超时返回 None。"""
    ser = getattr(robot, "_serial_port", None)
    if ser is None:
        return None
    try:
        ser.reset_input_buffer()
        ser.write(command + "\r")
        ser.flush()
    except Exception:
        return None
    buf = ""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            pending = ser.inWaiting()
            if pending:
                buf += ser.read(pending).decode("utf-8", "ignore")
                if token in buf and "]" in buf:
                    return buf
                if token in buf and token == "ENDSTOP":
                    return buf
        except Exception:
            return None
        time.sleep(0.02)
    return None


def _parse_floats(text):
    if not text:
        return None
    i, j = text.find("["), text.find("]")
    if i < 0 or j <= i:
        return None
    try:
        vals = [float(x) for x in text[i + 1:j].split(",") if x.strip()]
    except ValueError:
        return None
    return vals or None


# ---------------------------------------------------------------- 自检
class _FakeSerial:
    """最小串口替身：写入 G28 后按需回 ok，或故意不回应以验证超时。"""

    def __init__(self, answer=True):
        self.answer = answer
        self.inbuf = ""
        self.writes = []

    def write(self, data):
        cmd = data.decode() if isinstance(data, bytes) else data
        self.writes.append(cmd)
        if not self.answer:
            return
        if "G28" in cmd:
            self.inbuf = "ok\r"
        elif "M114" in cmd:
            self.inbuf = "COORDS[205.0,0.0,-40.0,0.0]\r\n"
        elif "M119" in cmd:
            self.inbuf = "ENDSTOP: 1,1,1\r\n"

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.inbuf = ""

    def inWaiting(self):
        return len(self.inbuf)

    def read(self, n):
        chunk, self.inbuf = self.inbuf[:n], self.inbuf[n:]
        return chunk.encode()


class _FakeRobot:
    def __init__(self, answer=True):
        self._serial_port = _FakeSerial(answer)
        self.calls = []

    def set_angle(self, i, a, s):
        self.calls.append(("set_angle", i, a, s))

    def set_mode(self, m):
        self.calls.append(("set_mode", m))

    def set_angles(self, a, s):
        self.calls.append(("set_angles", tuple(a), s))

    def power_on(self):
        self.calls.append(("power_on",))

    def release_all_servos(self):
        self.calls.append(("release_all_servos",))

    def set_gripper_state(self, v, s):
        self.calls.append(("set_gripper_state", v, s))

    def get_switch_state(self):
        return ["1", "1", "1"]


def motion_selftest():
    """纯软件自检：门禁、限位裁剪、步长截断、G28 超时。"""
    m = P340Motion(axes=3, limits_3=((-150, 170), (-20, 90), (-5, 75)),
                   limits_4=((-150, 170), (-20, 90), (-5, 110), (-179, 179)))

    # 1) 未回零一律拒绝
    for i in range(3):
        try:
            m.plan(i, 20.0)
        except MotionRejected:
            pass
        else:
            raise AssertionError("未回零仍然放行 J{}".format(i + 1))

    # 2) 回零后清零放行
    m.mark_homed()
    assert m.homed and m.joints == [0.0] * 4, "回零未清零"
    tgt, delta = m.plan(0, 30.0)
    assert (tgt, delta) == (30.0, 30.0), "J1 增量计算错误"
    m.commit(0, tgt)
    assert m.joints[0] == 30.0 and m.sent == 1

    # 3) 限位裁剪（J1 +170 / J2 +90 / J3 -5）：先把模型放到限位附近，
    #    避免被 MAX_STEP_DEG 截断提前吃掉结果
    m.joints = [160.0, 80.0, 5.0, 0.0]
    assert m.plan(0, 999.0)[0] == 170.0, "J1 上限未裁剪"
    assert m.plan(1, 999.0)[0] == 90.0, "J2 上限未裁剪"
    assert m.plan(2, -999.0)[0] == -5.0, "J3 下限未裁剪"

    # 4) 步长截断：一次目标跨 90°，实际只走 MAX_STEP_DEG
    m2 = P340Motion(axes=3, limits_3=((-150, 170), (-20, 90), (-5, 75)),
                    limits_4=((-150, 170), (-20, 90), (-5, 110), (-179, 179)))
    m2.mark_homed()
    tgt, delta = m2.plan(1, 90.0)
    assert abs(delta - MAX_STEP_DEG) < 1e-9 and abs(tgt - MAX_STEP_DEG) < 1e-9, \
        "步长未按 {}° 截断".format(MAX_STEP_DEG)

    # 5) 增量过小 / 轴号越界
    for bad in ((0, 0.0), (3, 10.0)):
        try:
            m2.plan(*bad)
        except MotionRejected:
            pass
        else:
            raise AssertionError("非法指令未拒绝: {}".format(bad))

    # 6) 4 轴模式解锁 J3 110 / J4 ±179（同样先把模型放到限位附近）
    m2.set_axes(4)
    m2.joints = [0.0, 0.0, 100.0, 160.0]
    assert m2.plan(2, 500.0)[0] == 110.0, "4 轴 J3 上限应为 110"
    assert m2.plan(3, 999.0)[0] == 179.0, "J4 上限应为 179"
    m2.set_axes(3)
    assert m2.plan(2, 200.0)[0] == 75.0, "切回 3 轴 J3 上限应回到 75"

    # 7) G28 有应答 / 无应答超时
    assert g28(_FakeRobot(True), timeout=0.5) is True, "G28 未识别 ok"
    t0 = time.time()
    assert g28(_FakeRobot(False), timeout=0.3) is False, "G28 无应答应超时"
    assert time.time() - t0 < 2.0, "G28 超时实现退化为长阻塞"

    # 8) 查询类指令必须在超时内返回，不能卡死采集线程
    robot = _FakeRobot(True)
    assert read_coords(robot, timeout=0.5) == [205.0, 0.0, -40.0, 0.0], "M114 解析错误"
    assert "1,1,1" in (switch_state(robot, timeout=0.5) or ""), "M119 解析错误"
    t0 = time.time()
    assert read_angles(_FakeRobot(False), timeout=0.3) is None, "M12 无应答应返回 None"
    assert read_coords(_FakeRobot(False), timeout=0.3) is None
    assert time.time() - t0 < 2.0, "查询超时实现退化为长阻塞"

    # 8) 模式切换
    for mode in (MODE_REL, MODE_ABS):
        m2.set_mode(mode)
        assert m2.mode == mode
    return True


__all__ = [
    "DEFAULT_SPEED", "HOME_TIMEOUT", "MAX_STEP_DEG", "MIN_STEP_DEG",
    "MODE_ABS", "MODE_LABELS", "MODE_REL", "MotionRejected", "P340Motion",
    "g28", "motion_selftest", "read_angles", "read_coords", "switch_state",
]
