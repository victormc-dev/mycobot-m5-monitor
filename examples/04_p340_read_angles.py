# coding=utf-8
"""只读示例：读取 ultraArm P340 的关节角与末端坐标，不产生任何运动。

    python examples/04_p340_read_angles.py --port COM3
    python examples/04_p340_read_angles.py --port COM3 --axes 4 --count 5

P340 使用 G-code 文本协议，读数接口为 get_angles_info() / get_coords_info()，
与六轴的 get_angles() / get_coords() 不同，故单独提供本示例。
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap  # noqa: E402  自动引导依赖（venv 优先，回退 vendor）
import robot_profiles as profiles  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="读取 ultraArm P340 关节角 / 坐标（只读）")
    ap.add_argument("--port", default="", help="串口号，如 COM3 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="P340 固定 115200")
    ap.add_argument("--axes", type=int, default=3, choices=(3, 4),
                    help="3 轴常规配置 / 4 轴选配，默认 3")
    ap.add_argument("--count", type=int, default=3, help="采样次数，默认 3")
    ap.add_argument("--interval", type=float, default=1.0, help="采样间隔（秒）")
    args = ap.parse_args()

    prof = profiles.get_profile("p340")
    print("型号:", profiles.summary(prof))
    print("读数接口:", prof.angle_api, "/", prof.coords_api, "| 自由度:", args.axes)

    if not args.port:
        print("请指定 --port；串口列表：python check_env.py --model p340")
        return 2

    ok, msg = profiles.driver_available(prof)
    if not ok:
        print("驱动不可用:", msg)
        return 1

    limits = prof.joint_limits_alt if args.axes == 4 else prof.joint_limits
    mc = profiles.create_robot(prof, args.port, args.baud)
    print("硬件提醒: 12V 5A 供电、串口 115200、无 Transponder/Atom 流程")

    for i in range(args.count):
        angles = profiles.read_angles(mc, prof)
        coords = profiles.read_coords(mc, prof)
        shown = angles[:args.axes] if angles else None
        over = profiles.over_limit(prof, shown, limits)
        warn = "  [超限位 J{}]".format(",".join(str(j) for j in over)) if over else ""
        print("[{}/{}] 关节角: {}  坐标: {}{}".format(
            i + 1, args.count, shown, coords, warn))
        time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    sys.exit(main())
