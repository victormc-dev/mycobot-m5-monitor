# coding=utf-8
"""型号适配层（新增模块，不改写原有 280 M5 实现）。

默认型号仍为 myCobot 280 M5，行为与适配前完全一致；本模块额外提供
myCobot 320 M5 与 ultraArm P340 的规格参数、驱动工厂与界面文案：

    joint_monitor.py --model 320 / --model p340
    check_env.py     --model 320 / --model p340
    examples/*.py    --model 320（P340 使用专属只读示例）

ultraArm P340 参数来源（大象机器人官方）：
  - 产品参数页  https://www.elephantrobotics.com/ultraarm-p340-spcn/
  - 产品手册    https://docs.elephantrobotics.com/docs/ultraArm/
  - Python API   3-HowToUseultraArm/2-SoftwareControl/4-Python/2-PythonAPI.html
  - 关节限位    官网表格 J1 -150~+170 / J2 -20~+90 / J3 -5~+75（3 轴）；
                驱动 ultraArmP340.set_angle() 文档另记 J3 -5~+110、J4 ±179（4 轴）
    读数接口：get_angles_info() / get_coords_info()（G-code 文本协议），与六轴不同。

320 M5 参数来源（大象机器人官方）：
  - 产品参数页  https://www.elephantrobotics.com/mycobot-320-m5-cn/parameter/
  - 机器规格参数 https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/
                 2-ProductFeature/2.1_320_M5_product/2.1.1-MachineSpecification.html
  - Python API   https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/
                 10-ApplicationBasePython/10.2_320_M5-ApplicationPython/2_API.html
    官方示例：from pymycobot import MyCobot320; mc = MyCobot320('COM3', 115200)

依赖说明：pymycobot 3.9.9（项目已固定版本，vendor/ 内已解包）同时内置
mycobot280.py 与 mycobot320.py，因此本次适配无需新增或升级任何依赖。
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass

DEFAULT_MODEL = "280"

# 官方 320 M5 关节运动范围：J1~J5 -165~+165，J6 -175~+175
LIMIT_320 = (
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-165.0, 165.0),
    (-175.0, 175.0),
)

TIP_FULL_320 = (
    "型号：myCobot 320 M5｜工作半径 350 mm｜负载 1 kg｜6 自由度\n"
    "关节限位：J1~J5 ±165°，J6 ±175°；超出限位会在状态栏提示。\n"
    "轴向：J1 基座 / J2 肩 / J3 肘 / J4 腕俯仰 / J5 腕翻滚 / J6 腕偏摆。\n"
    "姿态几何按 350/280 = 1.25 倍连杆比例绘制，仍为示意、非精确 DH 解算。\n\n"
    "真机接入：外接 DC 24V 电源（KPPX-4P 4 针接口），Type-C 连接 Basic；\n"
    "按官方 320 M5 首次使用流程烧录固件并确认串口转发就绪，波特率 115200。\n"
    "注意：320 M5 不是 12V 供电，也不要套用 280 的 Atom 固件流程。"
)
TIP_SHORT_320 = (
    "320 M5：J1~J5 ±165°、J6 ±175°；连杆按 1.25 倍绘制。"
    "24V 供电 / 115200 波特率；拖动旋转视角、滚轮缩放。非精确 DH 解算。"
)

LIMIT_CAPTION_320 = "显示量程 ±175；J1~J5 限位 ±165、J6 ±175（超限提示）"

# ultraArm P340：3 轴为出厂常规配置，第 4 轴为选配（官方标「自由度 3-4」）
LIMIT_P340_3AXIS = (
    (-150.0, 170.0),
    (-20.0, 90.0),
    (-5.0, 75.0),
)
LIMIT_P340_4AXIS = (
    (-150.0, 170.0),
    (-20.0, 90.0),
    (-5.0, 110.0),
    (-179.0, 179.0),
)
JOINT_NAMES_P340 = ("J1 基座回转", "J2 大臂", "J3 小臂", "J4 末端旋转（选配）")
COORD_NAMES_P340 = ("X", "Y", "Z", "E")

TIP_FULL_P340 = (
    "型号：ultraArm P340｜3~4 轴步进码垛臂｜工作半径 340 mm｜负载 650 g\n"
    "限位（3 轴）：J1 -150~+170°，J2 -20~+90°，J3 -5~+75°；\n"
    "4 轴模式：J3 -5~+110°，J4 -179~+179°（超限在状态栏提示）。\n"
    "供电 12V 5A，串口 115200；读数用 get_angles_info() / get_coords_info()。\n"
    "界面左列选择卡切到 P340 后启用四轴专用面板：角度条 / 趋势 / 坐标 / 平面姿态。\n\n"
    "注意：P340 不是六轴机械臂，不要套用 280/320 的 Transponder、Atom 固件与 24V 流程。"
)
TIP_SHORT_P340 = (
    "P340：3~4 轴步进码垛臂，12V 5A / 115200，四轴专用面板。"
    "选配第 4 轴可在面板内切换；姿态为平面几何示意。"
)
LIMIT_CAPTION_P340 = "P340 四轴面板：J1 -150~+170、J2 -20~+90、J3 -5~+75（超限提示）"


@dataclass(frozen=True)
class RobotProfile:
    """单个型号的静态适配参数。joint_limits 为 None 表示沿用原有行为。"""

    key: str
    label: str
    driver_module: str
    driver_class: str
    baud: int
    reach_mm: int
    payload_g: int | None
    weight_g: int | None
    repeatability: str | None
    power: str
    joint_limits: tuple | None
    display_limit: float
    geom_scale: float
    spec_url: str
    docs_url: str
    hardware_note: str
    tip_full: str | None = None      # None → 界面沿用 280 原文案（不改写）
    tip_short: str | None = None
    limit_caption: str | None = None
    # 以下为四轴/异构型号新增字段，均带默认值，280/320 行为完全不变
    dof: int = 6
    dof_options: tuple = ()
    max_joints: int = 6
    joint_names: tuple | None = None
    coord_names: tuple | None = None
    angle_api: str = "get_angles"
    coords_api: str = "get_coords"
    joint_limits_alt: tuple | None = None   # P340 选配 4 轴模式的限位


PROFILES = {
    "280": RobotProfile(
        key="280",
        label="myCobot 280 M5",
        driver_module="pymycobot.mycobot280",
        driver_class="MyCobot280",
        baud=115200,
        reach_mm=280,
        payload_g=None,             # 原项目未声明，保持不编造
        weight_g=None,
        repeatability=None,
        power="12V / 3-5A，DC 5.5x2.1（原项目 README）",
        joint_limits=None,          # 沿用原 ±180 显示量程，不做越界判定
        display_limit=180.0,
        geom_scale=1.0,
        spec_url="https://docs.elephantrobotics.com/docs/mycobot_280_m5_cn/",
        docs_url="https://docs.elephantrobotics.com/docs/mycobot_280_m5_cn/",
        hardware_note="外接 12V 电源；Basic 屏进 Transponder 按 A 显示 Atom: OK",
    ),
    "320": RobotProfile(
        key="320",
        label="myCobot 320 M5",
        driver_module="pymycobot.mycobot320",
        driver_class="MyCobot320",
        baud=115200,
        reach_mm=350,
        payload_g=1000,
        weight_g=3360,
        repeatability="±0.5 mm",
        power="DC 24V / 120W（官网参数页）；机规格页记 24V 9.2A；KPPX-4P 4 针插座",
        joint_limits=LIMIT_320,
        display_limit=175.0,
        geom_scale=1.25,            # 350 mm / 280 mm
        spec_url="https://www.elephantrobotics.com/mycobot-320-m5-cn/parameter/",
        docs_url="https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/",
        hardware_note=(
            "外接 DC 24V 电源，Type-C 连接 M5Stack-Basic；"
            "按官方 320 M5 首次使用流程烧录固件并确认串口转发就绪"
        ),
        tip_full=TIP_FULL_320,
        tip_short=TIP_SHORT_320,
        limit_caption=LIMIT_CAPTION_320,
    ),
    "p340": RobotProfile(
        key="p340",
        label="ultraArm P340",
        driver_module="pymycobot.ultraArmP340",
        driver_class="ultraArmP340",
        baud=115200,
        reach_mm=340,
        payload_g=650,
        weight_g=2900,
        repeatability="±0.1 mm",
        power="12V 5A DC（100-240VAC 适配器）",
        joint_limits=LIMIT_P340_3AXIS,
        display_limit=180.0,
        geom_scale=1.0,             # P340 使用独立四轴面板，不套用六轴 3D 几何
        spec_url="https://www.elephantrobotics.com/ultraarm-p340-spcn/",
        docs_url="https://docs.elephantrobotics.com/docs/ultraArm/",
        hardware_note="外接 12V 5A 电源；串口 115200；按官方 ultraArm 文档安装驱动与固件",
        dof=3,
        dof_options=(3, 4),
        max_joints=4,
        joint_names=JOINT_NAMES_P340,
        coord_names=COORD_NAMES_P340,
        angle_api="get_angles_info",
        coords_api="get_coords_info",
        joint_limits_alt=LIMIT_P340_4AXIS,
        tip_full=TIP_FULL_P340,
        tip_short=TIP_SHORT_P340,
        limit_caption=LIMIT_CAPTION_P340,
    ),
}

_ALIASES = {
    "280": "280", "280m5": "280", "mycobot280": "280", "mycobot-280-m5": "280",
    "320": "320", "320m5": "320", "mycobot320": "320", "mycobot-320-m5": "320",
    "p340": "p340", "ultraarm": "p340", "ultraarmp340": "p340",
    "ultraarm-p340": "p340", "p340-ultraarm": "p340",
}


def model_keys():
    """返回可选型号键，供 argparse choices / 界面下拉使用。"""
    return tuple(PROFILES.keys())


def get_profile(model=DEFAULT_MODEL) -> RobotProfile:
    """按型号名取配置；宽容处理大小写与常见写法。"""
    key = _ALIASES.get(str(model).strip().lower().replace("_", "-"))
    if key is None:
        raise ValueError("未知型号 {!r}，可选：{}".format(model, ", ".join(model_keys())))
    return PROFILES[key]


def driver_available(profile: RobotProfile):
    """只检查驱动类能否导入，不打开串口。返回 (是否可用, 说明)。"""
    try:
        mod = importlib.import_module(profile.driver_module)
        getattr(mod, profile.driver_class)
        return True, "{} 可用".format(profile.driver_class)
    except Exception as exc:
        return False, "{}: {}".format(type(exc).__name__, exc)


def create_robot(profile: RobotProfile, port: str, baud: int | None = None):
    """按型号动态创建驱动实例（懒导入，缺 320 模块不影响 280 使用）。"""
    try:
        mod = importlib.import_module(profile.driver_module)
        cls = getattr(mod, profile.driver_class)
    except Exception as exc:
        raise RuntimeError(
            "无法加载 {} 的驱动 {}.{}（{}: {}）。请确认 pymycobot>=3.9.9；"
            "离线模式检查 vendor/pymycobot/ 是否完整。".format(
                profile.label, profile.driver_module, profile.driver_class,
                type(exc).__name__, exc)
        ) from exc
    return cls(port, profile.baud if baud is None else baud)


def scale_geometry(base_h, link_len, grid_extent, scale):
    """按工作半径比例缩放可视化连杆与地面网格（280 的 scale=1.0 即原值）。"""
    return (
        float(base_h) * scale,
        tuple(float(x) * scale for x in link_len),
        float(grid_extent) * scale,
    )


def over_limit(profile: RobotProfile, angles, limits=None):
    """返回超出官方关节限位的关节编号列表（1~N）；280 未配置限位则返回 []。

    limits 可覆盖 profile.joint_limits（P340 选配 4 轴模式传 joint_limits_alt）。
    """
    lim = limits if limits is not None else profile.joint_limits
    if lim is None or not angles:
        return []
    out = []
    for i, (lo, hi) in enumerate(lim):
        try:
            v = float(angles[i])
        except (IndexError, TypeError, ValueError):
            continue
        if v < lo or v > hi:
            out.append(i + 1)
    return out


def read_angles(robot, profile: RobotProfile):
    """按型号调用正确的角度读取接口，统一返回 list[float]；失败返回 None。

    六轴用 get_angles()，ultraArm P340 用 get_angles_info()（G-code 文本协议）。
    """
    try:
        vals = getattr(robot, profile.angle_api)()
    except Exception:
        return None
    if not vals:
        return None
    try:
        return [float(x) for x in vals]
    except (TypeError, ValueError):
        return None


def read_coords(robot, profile: RobotProfile):
    """按型号调用正确的坐标读取接口，统一返回 list[float]；失败返回 None。"""
    try:
        vals = getattr(robot, profile.coords_api)()
    except Exception:
        return None
    if not vals:
        return None
    try:
        return [float(x) for x in vals]
    except (TypeError, ValueError):
        return None


def summary(profile: RobotProfile) -> str:
    """一行规格摘要，供自检与状态栏使用。"""
    payload = "{} g".format(profile.payload_g) if profile.payload_g else "未声明"
    return "{} | 工作半径 {} mm | 负载 {} | {} | {} baud | {}".format(
        profile.label, profile.reach_mm, payload,
        profile.power.split("（")[0], profile.baud, profile.driver_class)


__all__ = [
    "DEFAULT_MODEL", "PROFILES", "RobotProfile", "create_robot",
    "driver_available", "get_profile", "model_keys", "over_limit",
    "read_angles", "read_coords", "scale_geometry", "summary",
]
