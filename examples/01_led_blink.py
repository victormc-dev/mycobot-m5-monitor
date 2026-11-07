# coding=utf-8
"""官方文档 6.1.1「Python 简单使用」的 LED 闪烁示例（参数化 + 安全确认）。

    python examples\\01_led_blink.py --dry-run
    python examples\\01_led_blink.py --port COM3 --loops 3
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bootstrap  # noqa: E402  自动引导依赖
import robot_profiles as profiles  # noqa: E402  新增：280 / 320 型号适配层


def main():
    ap = argparse.ArgumentParser(description="myCobot 280 / 320 M5 末端 RGB 灯闪烁")
    ap.add_argument("--model", default=profiles.DEFAULT_MODEL, choices=profiles.model_keys(),
                    help="机械臂型号，默认 280；320 为新增适配")
    ap.add_argument("--port", default="", help="串口号，如 COM3（先跑 check_env.py 查看）")
    ap.add_argument("--baud", type=int, default=115200, help="M5 版本固定 115200")
    ap.add_argument("--loops", type=int, default=7, help="循环次数，默认 7")
    ap.add_argument("--dry-run", action="store_true", help="只验证依赖导入，不打开串口")
    ap.add_argument("--yes", action="store_true", help="跳过交互确认")
    args = ap.parse_args()
    prof = profiles.get_profile(args.model)

    print("依赖来源:", bootstrap.SOURCE)
    print("型号:", profiles.summary(prof))
    ok, msg = profiles.driver_available(prof)
    print("驱动:", msg)
    if not ok:
        return 1
    if args.dry_run:
        print("[dry-run] 未连接硬件，退出。")
        return 0

    if not args.port:
        print("请指定 --port，例如 --port COM3。串口列表：python check_env.py")
        return 2

    print("即将连接 {} @ {}；{}".format(
        args.port, args.baud,
        "确认串口转发就绪" if prof.key == "320"
        else "Basic 屏需处于 Transponder 且显示 Atom: OK"))
    if not args.yes:
        try:
            if input("输入 y 继续: ").strip().lower() != "y":
                print("已取消")
                return 0
        except EOFError:
            print("非交互环境，请追加 --yes")
            return 2

    mc = profiles.create_robot(prof, args.port, args.baud)
    i = args.loops
    while i > 0:
        mc.set_color(0, 0, 255)   # 蓝
        time.sleep(2)
        mc.set_color(255, 0, 0)   # 红
        time.sleep(2)
        mc.set_color(0, 255, 0)   # 绿
        time.sleep(2)
        i -= 1
    print("完成 {} 轮闪烁".format(args.loops))
    return 0


if __name__ == "__main__":
    sys.exit(main())
