# coding=utf-8
"""运动示例：回零 + J1 左右摆动 + 回零（会实际动作，谨慎使用）。

前置：底座已固定、机械臂运动范围内无障碍物、12V 电源已接、Transponder 已开启。

    python examples\\03_gohome.py --port COM3 --yes
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap  # noqa: E402
from pymycobot.mycobot280 import MyCobot280  # noqa: E402

ZERO = [0, 0, 0, 0, 0, 0]


def main():
    ap = argparse.ArgumentParser(description="myCobot 280 M5 回零与单关节摆动")
    ap.add_argument("--port", default="", help="串口号，如 COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=int, default=20, help="运动速度 0-100，默认 20")
    ap.add_argument("--angle", type=int, default=30, help="J1 摆动幅度，默认 ±30 度")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    args = ap.parse_args()

    if not args.port:
        print("请指定 --port；串口列表：python check_env.py")
        return 2

    print("[注意] 机械臂即将运动。请确认底座已固定、周围无障碍、供电正常。")
    if not args.yes:
        try:
            if input("输入 y 继续: ").strip().lower() != "y":
                print("已取消")
                return 0
        except EOFError:
            print("非交互环境，请追加 --yes")
            return 2

    mc = MyCobot280(args.port, args.baud)
    print("回零 ...")
    mc.send_angles(ZERO, args.speed)
    time.sleep(3)

    print("J1 +{} 度 ...".format(args.angle))
    mc.send_angle(1, args.angle, args.speed)
    time.sleep(3)

    print("J1 -{} 度 ...".format(args.angle))
    mc.send_angle(1, -args.angle, args.speed)
    time.sleep(3)

    print("再次回零 ...")
    mc.send_angles(ZERO, args.speed)
    time.sleep(3)
    print("完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
