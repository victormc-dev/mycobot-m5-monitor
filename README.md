# myCobot M5 Monitor

**myCobot 280 / 320 M5 六轴关节监控 + ultraArm P340 四轴码垛臂监控**。一套界面、一套代码，支持 **280 M5（默认）**、**320 M5**、**ultraArm P340** 三个型号；界面左列「机械臂型号」选择卡可随时切换。

> 仓库已由 `mycobot-280-m5-monitor` 更名为 **[mycobot-m5-monitor](https://github.com/victormc-dev/mycobot-m5-monitor)**。GitHub 自动重定向旧链接；已有本地克隆可执行
> `git remote set-url origin https://github.com/victormc-dev/mycobot-m5-monitor.git`
>
> 适配前的 280 M5 原版已发布为 [v1.0.0 Release](https://github.com/victormc-dev/mycobot-m5-monitor/releases/tag/v1.0.0)（快照 commit `77866ac`）。

---

## 一、支持型号

| 项目 | 280 M5（默认） | 320 M5 | ultraArm P340 |
|---|---|---|---|
| 类型 | 6 轴协作臂 | 6 轴协作臂 | **3~4 轴步进码垛臂** |
| Python 驱动 | `MyCobot280` | `MyCobot320` | `ultraArmP340` |
| 读数接口 | `get_angles()` | `get_angles()` | **`get_angles_info()`** |
| 串口波特率 | 115200 | 115200 | 115200 |
| 工作半径 / 负载 | 280 mm / 原项目未声明 | 350 mm / 1 kg | 340 mm / 650 g |
| 供电 | 12V / 3–5A，DC 5.5×2.1 | **DC 24V**（官网 120W / 文档 9.2A） | 12V 5A DC |
| 关节限位 | ±180 显示量程 | J1~J5 ±165°、J6 ±175° | J1 −150~+170°、J2 −20~+90°、J3 −5~+75° |
| 姿态视图 | 六轴 3D 球腕链 | 六轴 3D（连杆 ×1.25） | **独立四轴平面姿态面板** |
| 硬件流程 | Transponder / `Atom: OK` | 官方 320 首次使用流程 | 官方 ultraArm 文档，无 Transponder/Atom |

**不带 `--model` 运行即为 280 M5**，行为与 v1.0.0 逐行比对一致；320 与 P340 均为增量适配，**六轴渲染实现从未改写**。P340 因关节数与读数协议不同，使用 `p340_view.py` 独立面板。

## 二、快速开始

```bash
# 环境自检（不接硬件即可运行）
python check_env.py
python check_env.py --model 320
python check_env.py --model p340

# 启动监控界面（左列选择卡 / 状态栏下拉均可切换）
python joint_monitor.py
python joint_monitor.py --model 320 --demo
python joint_monitor.py --model p340 --demo    # 四轴独立面板

# 无界面自检：演示预设 / 运动学矩阵 / 320 与 P340 适配层断言
python joint_monitor.py --selftest

# 六轴示例（仅 280/320）
python examples/01_led_blink.py --model 320 --dry-run
python examples/02_read_angles.py --model 320 --port COM3

# P340 专属只读示例（--axes 3/4 对应官方 3-4 自由度）
python examples/04_p340_read_angles.py --port COM3
python examples/04_p340_read_angles.py --port COM3 --axes 4
```

Windows 下把 `python` 换成 `.\.venv\Scripts\python.exe`；一键建环境仍用原文的 `setup.ps1`。离线模式直接 `python` 即可——`vendor/` 已解包 `pymycobot 3.9.9` + `pyserial 3.5`，内含 `mycobot320.py` 与 `ultraArmP340.py`，**320 / P340 适配均零新增依赖**。前三个示例依赖六轴 API，不适用于 P340。

## 三、文件与文档

| 文件 | 说明 |
|---|---|
| `joint_monitor.py` | 主程序：三列界面、**左列型号选择卡**、六轴 3D 姿态、角度条 / 趋势、四种演示预设、自检 |
| `robot_profiles.py` | **型号真源**：三型号规格、驱动工厂、关节限位、读数接口、几何缩放、文案 |
| `p340_view.py` | **P340 四轴面板**：角度条 / 趋势 / 坐标 / 平面姿态 / 3-4 轴切换 |
| `check_env.py` | 依赖检查、串口扫描、可选联机只读测试（按型号选读数接口） |
| `examples/01~03` | 六轴示例：LED / 读角度 / 回零摆动（仅 280/320） |
| `examples/04_p340_read_angles.py` | P340 专属只读角度 / 坐标示例 |
| `README_320.md` / `README_P340.md` | 320 / P340 适配说明：参数对照、官方来源、边界与回滚 |
| `backup_280_original/` / `backup_before_p340/` | 两轮适配前的文件备份 |
| `vendor/` | 离线依赖（含 `mycobot320.py` 与 `ultraArmP340.py`） |

## 四、版本

| 版本 | 内容 |
|---|---|
| `v1.0.0` | 280 M5 原版（适配前快照），已发布 Release |
| `main` | 280 默认 + 320 / P340 增量适配，三型号并存 |

---

## 附录：myCobot 280 M5 原始环境文档（原文保留）

以下为适配前 `README.md` 的**全文，一字未改**，供 280 M5 的驱动安装、固件烧录、myStudio 使用与排错查阅。其中「Transponder / Atom 固件」等内容**仅适用于 280 M5**；320 M5 与 P340 请以上文表格及 `README_320.md` / `README_P340.md` 为准。

---

# myCobot 280 M5 本地运行环境

依据官方文档 [myCobot 280 M5 中文文档](https://docs.elephantrobotics.com/docs/mycobot_280_m5_cn/) 搭建，重点是 **6.1.1 环境搭建**、**5.2 myStudio 软件使用说明**、**4. 首次安装**。

---

## 一、当前状态

| 项目 | 状态 | 说明 |
|---|---|---|
| 本机 Python | ✅ 已存在 | `C:\Users\ThinkPad\AppData\Local\Programs\Python\Python313\python.exe`（3.13） |
| git / pip | ✅ 已存在 | git 位于 `C:\Program Files\Git\cmd\git.exe` |
| pymycobot | ✅ 离线包已就位 | `vendor/pymycobot` **3.9.9**，与 280 M5 2023 版文档配套，仅依赖 pyserial |
| pyserial | ✅ 离线包已就位 | `vendor/serial` **3.5** |
| `.venv` + pip 正式安装 | ✅ 已完成 | Python 3.13.7；`pymycobot 3.9.9` / `pyserial 3.5`（清华镜像） |
| 运行时验证 | ✅ 已通过 | `IMPORT_OK`、自检退出码 0、三个示例均可启动，详见下方验证记录 |
| 串口驱动 / 固件 / 机械臂联机 | ⏳ 需硬件侧操作 | 见「五、硬件前置条件」 |

> **说明**：离线包已解包到 `vendor/`，`bootstrap.py` 会自动把它加入 `sys.path`，因此**即使没有 `.venv`，用任意 Python 3 直接运行 `check_env.py` 和示例也能导入 `MyCobot280`**。
> **会话限制**：WorkBuddy 工具会话启动时会注入 4 个代理变量（`HTTP_PROXY`/`http_proxy`、`HTTPS_PROXY`/`https_proxy`，指向 `127.0.0.1:37860`）；注册表 HKCU/HKLM 中均无此项，**重启不会消除**。Windows 环境变量名不区分大小写，重复键使 .NET 环境块构建失败，会话内 `CreateProcess` 直接崩溃。已封装 `run_via_task.ps1` 按需绕过（见「八」）。**你自己开的终端 / PyCharm 完全不受影响**。

---

## 二、目录结构

```
MyCobot/
├─ README.md                 本说明
├─ requirements.txt          固定版本依赖（pymycobot==3.9.9 / pyserial==3.5）
├─ setup.ps1                 一键建立 .venv + 国内镜像安装 + 自检
├─ bootstrap.py              依赖引导：优先 venv，失败回退离线 vendor
├─ check_env.py              环境自检 + 串口扫描 + 可选联机测试
├─ examples/
│  ├─ 01_led_blink.py        官方 LED 闪烁示例（带参数、安全确认）
│  ├─ 02_read_angles.py      读取六个关节角与末端坐标（只读，最安全）
│  └─ 03_gohome.py           回零 + 单关节摆动（会动，需二次确认）
├─ vendor/                   离线依赖（wheel 解包，可直接 import）
└─ .wheels/                  下载的原始 wheel 与 4.0.7 备份（可删）
```

---

## 三、快速开始

### 1）环境已就绪，直接使用

```powershell
cd C:\Users\ThinkPad\Documents\MyCobot
.\.venv\Scripts\python.exe check_env.py
```

**验证记录（2026-11-04 实测）**

| 项目 | 结果 |
|---|---|
| Python | 3.13.7（`.venv`） |
| pip 安装 | `Successfully installed pymycobot-3.9.9 pyserial-3.5`，退出码 0 |
| 导入验证 | `IMPORT_OK pymycobot=3.9.9 pyserial=3.5`，`MyCobot280` 可用 |
| `check_env.py` | 结论「软件环境就绪 [OK]」，退出码 0，中文输出正常 |
| 三个示例 | 01 dry-run 退出码 0；02/03 未指定端口时正确提示并退出（不会误动作） |
| 串口扫描 | 未发现串口设备（机械臂尚未连接，属预期） |

安装与验证日志：`.wheels/setup_log.txt`、`.wheels/verify2_log.txt`。

### 2）重建环境（可选）

换 Python 版本或环境损坏时，在 **PowerShell** 中执行：

```powershell
cd C:\Users\ThinkPad\Documents\MyCobot
powershell -ExecutionPolicy Bypass -File .\setup.ps1
```

脚本会：创建 `.venv` → 用清华镜像 `pip install -r requirements.txt` → 自动运行 `check_env.py`。

### 3）离线模式（无需 `.venv`）

```powershell
python check_env.py
python examples\01_led_blink.py --dry-run
```

### 4）常用命令

```powershell
# 自检（含串口扫描）
.\.venv\Scripts\python.exe check_env.py

# 扫描到 COM3 后，联机测试（只读，不动作）
.\.venv\Scripts\python.exe check_env.py --port COM3

# LED 闪烁（官方示例，需 Transponder 已开启）
.\.venv\Scripts\python.exe examples\01_led_blink.py --port COM3 --loops 3

# 读取关节角 / 坐标
.\.venv\Scripts\python.exe examples\02_read_angles.py --port COM3
```

---

## 四、关键参数（务必核对）

| 项目 | 值 |
|---|---|
| Python 类 | `from pymycobot.mycobot280 import MyCobot280` |
| 串口 | 设备管理器 → 端口(COM 和 LPT) 查看，如 `COM3`；Linux 为 `/dev/ttyUSB0` |
| **波特率** | **115200**（M5 版本，写错会「程序正常但机械臂无反应」） |
| 供电 | **12V / 3–5A，DC 5.5×2.1**，必须外接官方电源；**仅插 Type-C 不足以驱动** |
| 底座 | 必须固定在平面吸盘底座 / G 型夹 / 牢固机座，否则运动会倾覆 |
| 屏幕功能 | 烧录后选择 **Transponder**，按 **A** 键，出现 `Atom: OK` 即转发通道就绪 |

---

## 五、硬件前置条件（官方流程）

### 1. 安装串口驱动

| 芯片 | 设备管理器显示 | Windows 驱动 |
|---|---|---|
| CP210X (CP2104) | `Silicon Labs CP210x USB to UART Bridge` | <https://download.elephantrobotics.com/software/drivers/CP210x_VCP_Windows.zip> |
| CP34X (CH9102) | `USB-Enhanced-SERIAL CH9102` | <https://download.elephantrobotics.com/software/drivers/CH9102_VCP_SER_Windows.exe> |
| 末端 Atom | — | <https://download.elephantrobotics.com/software/drivers/CDM21228_Setup.zip> |

> 不确定芯片时**两个都装**。装完重新插拔 USB，到设备管理器确认 COM 号。

### 2. 安装 myStudio 并烧录固件

- 下载：<https://www.elephantrobotics.com/download/> 或 <https://github.com/elephantrobotics/myStudio>（Windows 选 `.exe`，最新版 V3.5.8）
- **安装路径不能包含空格**
- 固件版本：

| 部位 | 固件 | 推荐版本 |
|---|---|---|
| M5Stack-Basic（底部） | miniRobot | **v2.1** |
| Atom（末端） | atomMain | 序列号 ≤ `ER28001202200415` 或无序列号 → **v4.1**；≥ `...0416` → **v5.1**（出厂默认已烧录，通常无需重烧） |

### 3. 上电与连接检测

1. 接 12V 电源 → 用 Type-C 连接 M5Stack-Basic 与 PC。
2. Basic 屏选择 **Information** → 按 **A**：查看 Atom 与六个电机连接状态；按 **B**：查看版本信息；按 **C**：退出。
3. 烧录完成后在 Basic 屏选择 **Transponder**，按 **A**，出现 `Atom: OK`。
4. 之后 PC 端 Python 才能通过串口控制机械臂。

---

## 六、常见问题

| 现象 | 处理 |
|---|---|
| `ModuleNotFoundError: No module named 'serial'` | 运行 `setup.ps1`；或确认 `vendor/serial` 存在、`bootstrap.py` 在同目录 |
| `SerialException: could not open port 'COM3'` | COM 号写错 / 被 myStudio 或串口助手占用，关闭占用程序 |
| 程序无报错但机械臂不动 | ① 检查波特率是否 115200；② Basic 是否处于 **Transponder** 且显示 `Atom: OK`；③ 是否接了 12V 电源 |
| 设备管理器看不到 COM 口 | 未装 CP210X / CP34X 驱动，或换一根支持数据的 Type-C 线 |
| PyCharm 里红色波浪线 | PyCharm → Settings → Project → Python Interpreter 指向 `.venv\Scripts\python.exe`，勾选 Inherit global site-packages |
| Python 3.13 下报兼容性错误 | 本机 3.13.7 已实测可用；若换环境后报错，按官方文档改装 **Python 3.10.7**（勾选 `Add Python to PATH`），再用 `setup.ps1 -PyExe <路径>` 重建 |
| WorkBuddy 会话内跑 python 报 `AccessViolationException` | 客户端注入的 4 个大小写重复代理变量所致，**注册表里没有、重启无效**；用 `.\run_via_task.ps1 -Command "..."` 代跑，或直接在自己的终端 / PyCharm 里运行。属 WorkBuddy 客户端问题，建议向官方反馈 |

---

## 七、版本说明

- 当前固定 **pymycobot 3.9.9 + pyserial 3.5**：与 280 M5（2023）文档配套，`MyCobot280` API 一致，顶层只依赖 pyserial，轻量且可复现。
- 官方 `pip install pymycobot --upgrade` 会装到 **4.0.7**，该版本顶层额外依赖 `numpy`、`bleak`，代码里 `from pymycobot.mycobot280 import MyCobot280` 仍然可用。如需升级：

```powershell
.\.venv\Scripts\python.exe -m pip install "pymycobot==4.0.7" -i https://pypi.tuna.tsinghua.edu.cn/simple
```

---

## 八、会话内执行器 `run_via_task.ps1`

仅当需要在 **WorkBuddy 工具会话内**代跑命令时使用（该会话无法直接创建子进程）：

```powershell
cd C:\Users\ThinkPad\Documents\MyCobot
.\run_via_task.ps1 -Command ".venv\Scripts\python.exe check_env.py --port COM3"
.\run_via_task.ps1 -Command ".venv\Scripts\python.exe examples\01_led_blink.py --port COM3" -TimeoutSec 120
```

原理：命令写入临时 `.cmd` → 按需注册 Windows 计划任务 → 由任务计划服务（环境干净，无注入代理变量）创建进程 → 输出重定向到 `.wheels/runner_log.txt` 并回显 → **自动注销任务**，系统不留常驻项。

实测（2026-11-04 重启后）：`.venv` 完好、Python 3.13.7、`check_env.py` 退出码 0、结论「软件环境就绪 [OK]」；`PORT_COUNT=0`，机械臂尚未连接。

---

## 九、关节角度实时监控软件 `joint_monitor.py`

**双击 `start_monitor.cmd`** 即可启动；扫不到串口会自动进入演示模式并**自动开始六轴动画**，无需点任何按钮：

```powershell
cd C:\Users\ThinkPad\Documents\MyCobot
.\.venv\Scripts\python.exe joint_monitor.py          # 自动
.\.venv\Scripts\python.exe joint_monitor.py --demo   # 强制演示
```

| 功能 | 说明 |
|---|---|
| 六关节实时角度 | J1~J6 数值（`{:+.2f}`）+ 双向彩色条形，显示量程 ±180° |
| 角度趋势曲线 | 最近 120 帧、六色折线，便于观察运动过程 |
| **独立纵列 3D 几何演示** | **J1~J6 全部参与**：3x3 旋转矩阵串联运动链 + 正交投影线框；**独占中间纵列、高度铺满主区**，每个关节一个彩色轴环，深度排序绘制 |
| 动作预设 | `自由摆动` / `逐关节演示` / `抓取循环` / `归零静止`，切换即时生效 |
| 逐关节演示 | 每 2.5 秒只摆动一个关节（J1→J6 循环），轴环高亮 + 顶部指示条标注，便于逐个观察 |
| 视角操作 | 左键拖动旋转、滚轮缩放、双击或「复位视角」按钮复位 |
| 流畅度 | 3D 独立于串口采样率以 **~30 FPS 插值重绘**（最小角差、±180° 规范），低速采样也不卡 |
| 末端坐标 | X/Y/Z/Rx/Ry/Rz，真机每 5 帧读取一次 `get_coords()` |
| 连接配置 | 串口下拉自动扫描、波特率默认 **115200**、采样间隔 100/200/500/1000 ms |
| 线程模型 | 后台线程读串口 + 队列投递，主线程 40 ms 轮询刷新，UI 不卡 |
| **三列布局** | 左：六关节数值 / 中：**3D 几何演示（主列，weight 6）** / 右：趋势 + 坐标；列权重 1:6:2，3D 列始终最宽 |
| **三列等高对齐** | 中列与右列画布高度都按 `宿主高度 - 各自固定开销` 计算，**三列底边对齐**（实测留白 8px，无裁切）；高趋势画布自动加密网格并加 ±180 刻度与时间轴 |
| **窗口自适应** | 启动时按屏幕工作区选初始尺寸并居中；缩放时所有画布按实际尺寸重绘，3D 按包围盒自动缩放居中，机械臂始终完整可见 |
| 窄/矮窗口兜底 | **工具栏保持单行**（紧凑控件 + 尾部弹性列，880 宽仍有 24px 余量）、说明文字矮窗口自动精简、主区过矮时自动出现纵向滚动条；`minsize` 880×600 |

**关节映射（按官方 URDF 的 Wrist Pitch / Roll / Yaw 球腕校准）**：

| 关节 | 名称 | 旋转轴（局部） |
|---|---|---|
| J1 | 基座回转 | 世界 Z |
| J2 / J3 | 肩俯仰 / 肘俯仰 | 局部 Y（共面，平面臂的正常特性） |
| J4 | 腕俯仰 Wrist Pitch | 局部 **X**，垂直于肩肘轴 |
| J5 | 腕翻滚 Wrist Roll | 局部 **Z**（连杆轴） |
| J6 | 腕偏摆 Wrist Yaw | 局部 **Y**，垂直于 J5 |

腕部 J4/J5/J6 三轴**两两垂直 90°**。J5 关节点处画横销、J6 末端画两片夹爪，让这两个关节的运动可见；底座另有随 J1 转动的指针与静止地面网格作参照。轴依据 `mycobot_ros` 的 `mycobot_280m5_with_gripper_parallel.urdf`（六个 revolute joint 的 `origin rpy` 为 ±90° 交替）。连杆长度为可视化取值、腕部已收紧为 30/18/16，**非精确 DH 参数解算**，不可用于碰撞检测或轨迹规划。

> 2026-11-04 修正记录：旧版把 J2/J3/J4 都画成绕局部 Y（J4 成了「第三个肘」），J5/J6 又同轴自转，6 自由度实际退化；且 J5 横销误画在 J6 关节点。已按上表重建，并把该几何约束写成 `--selftest` 回归断言 `AXIS_OK`。

**三列布局实现**：`body` 用 grid 三列（minsize 225/380/235，weight 1/6/2）。3D 卡片独占中列；`_on_host_configure` 令 `h_pose = 宿主高度 - POSE_CHROME(80)`、`h_trend = 宿主高度 - RIGHT_CHROME(212)`，两列**各自铺满、底边对齐**；`RIGHT_CHROME` 按实测裁切像素校准（初版 182 时右列超出 22px，调到 212 后留 8px 余量）。趋势画布拉高后，`_draw_trend()` 按高度自动加密网格（5/7/9 条）、补 ±180/±90/0 刻度与「-120 帧 / 现在」时间轴，避免中间空荡。坐标卡片改 3 行 2 列以适配窄右列。各画布的**初始请求宽度刻意调小**（bar 200 / trend 220 / pose 280），否则子控件请求宽度会撑住列、让 weight 失效；绘图一律按 `winfo_width()/winfo_height()` 动态取值，所以调小请求宽度不影响显示。左列说明文字固定 `wraplength=200` —— 动态加大 wraplength 会反向撑宽左列形成棘轮，压缩 3D 主列。

**窗口自适应实现**：主区放在可滚动的 Canvas 宿主里，`_after_resize` 防抖 80 ms 后重绘并按控件真实请求高度校正。`_fit_pose()` 把当前姿态全部 3D 点（含地面网格四角）单位投影求包围盒，据此算缩放与居中，系数平滑跟随、resize 时立即贴合；画布变大后机械臂随之放大，滚轮缩放作为用户主动覆盖。顶部关节指示条在窄画布自动只显示编号，底部提示自动精简。

> 效果对比：改三列前 3D 画布约 **535×305**（16.3 万 px²）；改后 1160×900 窗口为 **527×625**（32.9 万 px²，约 2 倍），1400×1000 为 **688×625**（43 万 px²）。

**真机使用**：装好驱动、接 12V 电源、Basic 屏进 **Transponder** 按 A 出 `Atom: OK` → 取消勾选「演示模式」→ 选 COM 口 → 「开始采集」。读数一直为 `--` 时，检查 COM 口是否被 myStudio 占用、波特率是否 115200。

**自检**（供自动化验证）：

```powershell
.\.venv\Scripts\python.exe joint_monitor.py --selftest    # 无界面：预设、3D 数学、插值
.\.venv\Scripts\python.exe joint_monitor.py --pose-test   # 3D 绘制 + 多尺寸自适应 + 无裁切断言
```

实测 2026-11-04（日志 `.wheels/align_log2.txt`，三列等高对齐后）：`py_compile` 退出码 0；`PRESET_OK x4`、`LERP_OK`、`AXIS_OK wrist_orthogonal=True cross_bar_on_J5=True`、`POSE3D_OK segments=6 joints=6`、`SELFTEST_OK`；`--pose-test` 输出 `TK_OK POSE_DRAW_OK`、`FIT_OK cases=54 sizes=6 all_points_inside=True`、`SCROLL_OK`、`VERTICAL_OK distinct_heights=[380, 480, 625]`、`CLIP_OK no_overflow cases=5 worst_f3=-8 worst_tip=-13`、`TOOLBAR_OK single_row cases=5 min_slack=24`、`POSE_COLUMN_OK min_height=380 max_area=430000px2`、`ALIGN_OK cases=4 worst_gap=8px`。1160×900 下三列宽 `260/553/307`、3D 画布 `527×625`、趋势画布 `281×493`；1400×1000 下 `688×625` / `334×493`。同一日志 `AXIS_CHECK` 段复核 J3-J4 / J4-J5 / J5-J6 均 **90.0°**、J2-J3 0.0°、横销距 J5 关节 0.0。全部退出码 0。
