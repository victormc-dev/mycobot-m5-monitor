# coding=utf-8
"""myCobot 280 M5 环境自检。

用法：
    python check_env.py                      # 只检查软件环境 + 扫描串口
    python check_env.py --port COM3          # 追加联机只读测试
    python check_env.py --port COM3 --baud 115200
"""

import argparse
import platform
import sys
from pathlib import Path

# 不强制 reconfigure：Windows 控制台由 PEP 528 用 UTF-8 输出，
# 重定向到文件/管道时则跟随系统 ANSI(cp936)，两种场景中文都能正常显示。
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bootstrap  # noqa: E402


def show(key, value):
    print("  {:<22} {}".format(key, value))


def main():
    ap = argparse.ArgumentParser(description="myCobot 280 M5 环境自检")
    ap.add_argument("--port", default="", help="串口号，如 COM3 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，M5 固定 115200")
    args = ap.parse_args()

    print("=" * 62)
    print("myCobot 280 M5  本地运行环境自检")
    print("=" * 62)

    print("[1] Python")
    show("版本", sys.version.split()[0])
    show("解释器", sys.executable)
    show("系统", platform.platform())
    show("依赖来源", bootstrap.SOURCE)

    print("[2] 依赖包")
    ok = True
    try:
        import pymycobot
        from pymycobot.mycobot280 import MyCobot280
        import serial
        from serial.tools import list_ports

        show("pymycobot", pymycobot.__version__)
        show("pyserial", serial.VERSION)
        show("MyCobot280", "OK")
    except Exception as exc:
        ok = False
        show("ERROR", "{}: {}".format(type(exc).__name__, exc))
        print("\n结论：环境未就绪，请运行 setup.ps1 或检查 vendor/ 目录。")
        return 1

    print("[3] 串口扫描")
    ports = list(list_ports.comports())
    if not ports:
        show("结果", "未发现串口设备")
        print("       -> 检查 12V 电源、Type-C 数据线、CP210X/CP34X 驱动")
    for p in ports:
        desc = (p.description or "").upper()
        chip = ""
        if "CP210" in desc:
            chip = "  [CP210X 驱动]"
        elif "CH9102" in desc or "USB-ENHANCED" in desc:
            chip = "  [CP34X 驱动]"
        show(p.device, "{} {}".format(p.description or "-", chip))

    print("[4] 硬件状态提醒")
    show("波特率", str(args.baud) + "  (M5 版本必须为 115200)")
    show("供电", "12V / 3-5A / DC 5.5x2.1，不能只靠 Type-C")
    show("Basic 屏", "Transponder 功能，按 A 显示 Atom: OK")

    if args.port:
        print("[5] 联机只读测试  {}".format(args.port))
        try:
            mc = MyCobot280(args.port, args.baud)
            angles = mc.get_angles()
            coords = mc.get_coords()
            show("关节角", angles)
            show("末端坐标", coords)
            show("结果", "串口已打开" + ("，机械臂有响应" if angles else "，但未收到数据（检查 Transponder / 固件）"))
        except Exception as exc:
            ok = False
            show("ERROR", "{}: {}".format(type(exc).__name__, exc))
            print("       -> 检查 COM 号、是否被 myStudio 占用、驱动是否安装")
    else:
        print("[5] 联机测试")
        show("结果", "未指定 --port，跳过（可加 --port COM3 重试）")

    print("=" * 62)
    print("结论：" + ("软件环境就绪 [OK]" if ok else "存在异常，请按上方提示处理 [FAIL]"))
    print("=" * 62)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
