# coding=utf-8
"""型号适配层（新增模块，不改写原有 280 M5 实现）。

默认型号仍为 myCobot 280 M5，行为与适配前完全一致；本模块额外提供
myCobot 320 M5 的规格参数、驱动工厂与界面文案，供以下入口共同使用：

    joint_monitor.py --model 320
    check_env.py     --model 320
    examples/*.py    --model 320

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
}

_ALIASES = {
    "280": "280", "280m5": "280", "mycobot280": "280", "mycobot-280-m5": "280",
    "320": "320", "320m5": "320", "mycobot320": "320", "mycobot-320-m5": "320",
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


def over_limit(profile: RobotProfile, angles):
    """返回超出官方关节限位的关节编号列表（1~6）；280 未配置限位则返回 []。"""
    if profile.joint_limits is None or not angles:
        return []
    out = []
    for i, (lo, hi) in enumerate(profile.joint_limits):
        try:
            v = float(angles[i])
        except (IndexError, TypeError, ValueError):
            continue
        if v < lo or v > hi:
            out.append(i + 1)
    return out


def summary(profile: RobotProfile) -> str:
    """一行规格摘要，供自检与状态栏使用。"""
    payload = "{} g".format(profile.payload_g) if profile.payload_g else "未声明"
    return "{} | 工作半径 {} mm | 负载 {} | {} | {} baud | {}".format(
        profile.label, profile.reach_mm, payload,
        profile.power.split("（")[0], profile.baud, profile.driver_class)


__all__ = [
    "DEFAULT_MODEL", "PROFILES", "RobotProfile", "create_robot",
    "driver_available", "get_profile", "model_keys", "over_limit",
    "scale_geometry", "summary",
]
