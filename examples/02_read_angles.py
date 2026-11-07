# coding=utf-8
"""只读示例：读取六个关节角与末端坐标，不产生任何运动。

    python examples\\02_read_angles.py --port COM3
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap  # noqa: E402
import robot_profiles as profiles  # noqa: E402  新增：280 / 320 型号适配层


def main():
    ap = argparse.ArgumentParser(description="读取 myCobot 280 / 320 M5 关节角 / 坐标")
    ap.add_argument("--model", default=profiles.DEFAULT_MODEL, choices=("280", "320"),
                    help="六轴型号 280 / 320；P340 请用 04_p340_read_angles.py")
    ap.add_argument("--port", default="", help="串口号，如 COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=3, help="采样次数，默认 3")
    ap.add_argument("--interval", type=float, default=1.0, help="采样间隔（秒）")
    args = ap.parse_args()

    if not args.port:
        print("请指定 --port；串口列表：python check_env.py")
        return 2

    prof = profiles.get_profile(args.model)
    print("型号:", profiles.summary(prof))
    ok, msg = profiles.driver_available(prof)
    if not ok:
        print("驱动不可用:", msg)
        return 1
    mc = profiles.create_robot(prof, args.port, args.baud)
    for i in range(args.count):
        angles = mc.get_angles()
        coords = mc.get_coords()
        print("[{}/{}] 关节角: {}  末端坐标: {}".format(i + 1, args.count, angles, coords))
        time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    sys.exit(main())
