#!/bin/bash
# coding=utf-8
# myCobot / ultraArm 关节监控 · macOS 一键启动器
#
# 双击本文件即可在「终端」中启动界面；也可在命令行传参：
#   ./start_monitor.command                        双击等同，默认 280 M5
#   ./start_monitor.command --model p340           直接进 P340 四轴面板
#   ./start_monitor.command --model 320 --demo     演示模式（无需硬件）
#   ./start_monitor.command --check --model p340   只跑环境自检，不开界面
#   ./start_monitor.command --setup                一次性建 .venv（清华镜像）
#   ./start_monitor.command --dry-run              只打印将执行的命令
#   ./start_monitor.command --help                 显示本帮助
#
# 型号不必在命令行指定：界面左列「型号」选择卡 / 状态栏下拉可随时切换。
# Python 探测顺序：.venv → PATH → Homebrew / 系统 → WorkBuddy 内置 Python。
# 没有 .venv 时由 bootstrap.py 自动回退到 vendor/ 离线依赖，零配置即可运行。

cd "$(dirname "$0")" || exit 1

MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple"
PY=""

find_python() {
    if [ -x ".venv/bin/python" ]; then
        PY=".venv/bin/python"
        return 0
    fi
    for c in python3 python3.13 python3.12 python3.11 python3.10 python; do
        if command -v "$c" >/dev/null 2>&1; then
            PY="$(command -v "$c")"
            return 0
        fi
    done
    for p in /opt/homebrew/bin/python3 /usr/local/bin/python3 /usr/bin/python3; do
        if [ -x "$p" ]; then
            PY="$p"
            return 0
        fi
    done
    for p in "$HOME"/.workbuddy/binaries/python/versions/*/bin/python3; do
        if [ -x "$p" ]; then
            PY="$p"
            return 0
        fi
    done
    return 1
}

usage() {
    cat <<'USAGE'
myCobot / ultraArm 关节监控 · macOS 启动器

用法：
  ./start_monitor.command [选项] [传给 joint_monitor.py 的参数]

选项：
  --model 280|320|p340   指定型号（默认 280；界面内也可切换）
  --demo                 演示模式，无需硬件（透传给主程序）
  --check                只运行 check_env.py 环境自检
  --setup                一次性创建 .venv 并用清华镜像安装依赖
  --dry-run              只打印将执行的命令，不启动
  -h, --help             显示本帮助

示例：
  ./start_monitor.command
  ./start_monitor.command --model p340 --demo
  ./start_monitor.command --check --model 320
  ./start_monitor.command --setup

首次使用若提示没有执行权限：chmod +x start_monitor.command
USAGE
}

ACTION="start"
MODEL=""
DRY_RUN=0
EXTRA=()

while [ $# -gt 0 ]; do
    case "$1" in
        --model)
            shift
            MODEL="${1:-}"
            ;;
        --model=*)
            MODEL="${1#*=}"
            ;;
        --setup)
            ACTION="setup"
            ;;
        --check)
            ACTION="check"
            ;;
        --dry-run)
            DRY_RUN=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            EXTRA+=("$1")
            ;;
    esac
    shift
done

case "$MODEL" in
    ""|280|320|p340) ;;
    *)
        echo "错误: 未知型号 '$MODEL'（可选 280 / 320 / p340）"
        exit 2
        ;;
esac

if ! find_python; then
    echo "错误: 找不到 Python 3。请安装官方 Python 或执行 brew install python3 后重试。"
    echo "      也可先运行 ./start_monitor.command --setup 创建虚拟环境。"
    printf '按回车键关闭窗口...'
    read -r _
    exit 1
fi

if [ ! -f joint_monitor.py ]; then
    echo "错误: 当前目录下未找到 joint_monitor.py（脚本应与主程序放在同一目录）。"
    printf '按回车键关闭窗口...'
    read -r _
    exit 1
fi

# tkinter 只有启动图形界面时才必需；--check / --setup 不依赖它
if [ "$ACTION" = "start" ]; then
    if ! "$PY" -c 'import tkinter' >/dev/null 2>&1; then
        echo "警告: 当前 Python ($PY) 未提供 tkinter，图形界面可能无法启动。"
        echo "      建议安装 python.org 官方 Python，或执行 brew install python-tk。"
    fi
fi

if [ "$ACTION" = "setup" ]; then
    if [ -d .venv ]; then
        echo ".venv 已存在，跳过创建。"
    else
        echo "创建虚拟环境 .venv ..."
        "$PY" -m venv .venv || { echo "venv 创建失败"; exit 1; }
        echo "从清华镜像安装依赖（pymycobot 3.9.9 / pyserial 3.5）..."
        env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
            -u ALL_PROXY -u all_proxy \
            ./.venv/bin/python -m pip install --upgrade pip -i "$MIRROR" || exit 1
        env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy \
            -u ALL_PROXY -u all_proxy \
            ./.venv/bin/python -m pip install -r requirements.txt -i "$MIRROR" || exit 1
        echo "依赖安装完成。"
    fi
    echo "运行环境自检："
    ./.venv/bin/python check_env.py
    rc=$?
    if [ "$rc" -ne 0 ]; then
        printf '按回车键关闭窗口...'
        read -r _
    fi
    exit "$rc"
fi

if [ "$ACTION" = "check" ]; then
    CMD=("$PY" check_env.py)
else
    CMD=("$PY" joint_monitor.py)
fi
if [ -n "$MODEL" ]; then
    CMD+=(--model "$MODEL")
fi
for a in "${EXTRA[@]}"; do
    CMD+=("$a")
done

if [ "$DRY_RUN" -eq 1 ]; then
    echo "Python : $PY"
    printf '命令   : '
    printf '%s ' "${CMD[@]}"
    printf '\n'
    echo "工作目录: $(pwd)"
    exit 0
fi

echo "=========================================================="
echo " 启动中 · 型号 ${MODEL:-280（界面内可切换）}"
echo " Python : $PY"
echo " 目录   : $(pwd)"
echo " 关闭监控窗口即可退出；不要直接关掉终端再重开（串口会被占用）"
echo "=========================================================="

"${CMD[@]}"
rc=$?
if [ "$rc" -ne 0 ]; then
    echo
    echo "程序退出码 $rc，请查看上方错误信息。"
    printf '按回车键关闭窗口...'
    read -r _
fi
exit "$rc"
