# ultraArm P340 适配说明（增量，UI 型号选择卡）

本项目原本只支持 myCobot 280 M5，随后新增 320 M5。本次按[官方产品页](https://www.elephantrobotics.com/ultraarm-p340/cn)与[官方参数页](https://www.elephantrobotics.com/ultraarm-p340-spcn/)**新增 ultraArm P340**，采取「**UI 里增加型号选择卡 + 独立四轴面板**」的方式：

- **默认仍是 280 M5**，六轴主视图、渲染逻辑、演示预设、数据源**一行未改**；
- 界面左列顶部「**机械臂型号**」选择卡（280 M5 / 320 M5 / P340）与状态栏下拉联动；
- 选 **P340** 时，主内容区切到独立的四轴面板（角度条 / 趋势 / 坐标 / 平面姿态示意）；
- 选回 280 / 320 时切回原六轴视图。命令行也可 `python joint_monitor.py --model p340` 直接进入。

> 为什么不做成「六轴界面填空两轴」：P340 是 **3~4 轴步进码垛臂**，读数走 G-code 文本协议，关节限位非对称、运动学是平面连杆，与六轴球腕完全不同。硬塞进六轴视图会得到错误的姿态和坐标语义。

---

## 一、官方参数

| 项目 | ultraArm P340 | 备注 |
|---|---|---|
| 自由度 | **3~4**（第 4 轴选配） | 官方参数表 |
| 有效负载 | 650 g | |
| 最大工作半径 | 340 mm | |
| 重复定位精度 | ±0.1 mm | 步进电机 |
| 净重 / 寿命 | 2.9 kg / 5000 h | |
| 输入电源 | **12V 5A DC**（100-240VAC 适配器） | 与 280 同为 12V，但电流不同 |
| 关节范围（3 轴） | J1 −150~+170°、J2 −20~+90°、J3 −5~+75° | 官方参数表 |
| 关节范围（4 轴） | J3 −5~+110°、J4 −179~+179° | `ultraArmP340.set_angle()` 文档字符串 |
| 关节速度 / 末端速度 | J1~J3 100°/s / 100 mm/s | |
| 串口波特率 | **115200** | `ultraArmP340(port, baudrate=115200)` |
| 驱动类 | `pymycobot.ultraArmP340.ultraArmP340` | vendor 3.9.9 已内置 |
| 读数接口 | **`get_angles_info()` / `get_coords_info()`** | 返回 `ANGLES[...]` / `COORDS[...]` 文本解析结果 |
| 运动接口 | `set_angles()` / `set_angle()` / `go_zero()` / `set_gripper_state()` | 本项目只读，不调用运动接口 |

> **限位两处不一致**：官网参数表（3 轴）记 J3 上限 75°，驱动源码 `set_angle()` 文档记 J3 上限 110°、J4 ±179°（4 轴配置）。本项目按官方「自由度 3-4」提供面板内切换：3 轴用 75° 上限，4 轴用 110° + J4 ±179°。以设备实际配置和固件为准。

**依赖零变化**：`pymycobot==3.9.9` 的 `vendor/` 内已含 `ultraArmP340.py`，无需升级或新增 wheel。

---

## 二、界面与交互

| 位置 | 内容 |
|---|---|
| 左列顶部 | **机械臂型号选择卡**：单行紧凑卡片（型号标签 + 280/320/P340 三个按钮）；切换时状态栏显示当前型号规格摘要 |
| 状态栏右侧 | 型号下拉（与选择卡联动，采集中禁止切换） |
| P340 面板顶部 | 标题、规格摘要、**3 轴 / 4 轴** 切换（对应官方 3-4 自由度） |
| P340 面板左列 | 4 行角度条，**0 线固定居中**；左半区按 \|下限\|、右半区按上限分别缩放，正值向右 / 负值向左，非对称限位不再把 0 线挤偏；超限条变红并在状态栏提示，行标题标注 `lo~hi` |
| P340 面板中列 | **3D 姿态（默认）**：正交投影串联链、静止地面网格、四关节彩色轴环、深度排序连杆、顶部关节指示条；可一键切「2D 侧视」对照原平面示意 |
| P340 面板右列 | 角度趋势（按各自限位归一）+ 末端坐标 X/Y/Z/E（真机每 5 帧刷新） |
| 工具栏 | 复用原有串口 / 波特率 / 采样 / 演示模式 /「开始采集」按钮，P340 模式下委托给四轴面板；**「动作」下拉的四个演示预设均已适配四轴** |
| 3D 视角交互 | 画布内拖动改方位/俯仰、滚轮缩放、双击或「复位视角」按钮复位；自动按包围盒缩放保证全部图元可见；逐关节演示时对应轴环与指示条高亮 |

**演示预设**（主界面「动作」下拉，四项均已适配四轴，3 / 4 轴通用，采集中可实时切换）：

| 预设 | P340 行为 |
|---|---|
| 自由摆动 | 各轴不同频率联动，离线查看四轴姿态 |
| 逐关节演示 | 每 2.5 秒只摆动一个关节（3 轴轮 J1~J3、4 轴轮 J1~J4），状态栏与数值标签高亮当前关节 |
| 抓取循环 | 8 秒周期模拟码垛抓取动作 |
| 归零静止 | 全部关节回 0° |

四组角度均落在官方限位内（J1 ±80°、J2 −5~75°、J3 5~65°、J4 ±90°），无需硬件即可运行。

---

## 三、使用方法

```bash
# 命令行直接进 P340（也可启动后点选择卡）
python joint_monitor.py --model p340
python joint_monitor.py --model p340 --demo

# 环境自检与只读示例
python check_env.py --model p340
python check_env.py --model p340 --port COM3
python examples/04_p340_read_angles.py --port COM3
python examples/04_p340_read_angles.py --port COM3 --axes 4 --count 5

# 无界面自检（含 P340 驱动与限位断言）
python joint_monitor.py --selftest
```

Windows 下把 `python` 换成 `.\.venv\Scripts\python.exe`。
**macOS**：双击 `start_monitor.command`，或 `./start_monitor.command --model p340 --demo` 直接进四轴面板（首次先 `chmod +x start_monitor.command`）。

**硬件前置**：接 12V 5A 电源 → USB/串口连接 → 按[官方 ultraArm 文档](https://docs.elephantrobotics.com/docs/ultraArm/)安装驱动、烧录固件。**P340 没有 Transponder / `Atom: OK` 流程，也不要用 320 的 24V 电源。**

---

## 四、文件清单

**新增**
- `p340_view.py`：四轴监控面板（`P340Panel`：数据源、3/4 轴、限位归一、平面姿态）
- `examples/04_p340_read_angles.py`：P340 只读角度/坐标示例
- `README_P340.md`：本说明
- `backup_before_p340/`：本轮改动前的文件备份

**增量接入（原逻辑保留）**
- `robot_profiles.py`：新增 P340 profile，`RobotProfile` 扩展 `dof / dof_options / joint_names / coord_names / angle_api / coords_api / joint_limits_alt` 等默认字段；新增 `read_angles()` / `read_coords()` 统一读数入口；`over_limit()` 支持传入替代限位
- `joint_monitor.py`：左列型号选择卡、`_switch_view()`（Canvas window 切换）、`_start/_stop` 委托、P340 模式跳过六轴 3D 绘制、自检断言；**六轴渲染与数据源未改**
- `check_env.py`：`--model p340`，联机读数改走 `profiles.read_angles/read_coords`，硬件提示按型号区分
- `examples/01~03`：型号 choices 限定 280/320，防止把六轴运动示例误用于 P340
- `README.md`：总览升级为三型号，**原 280 全文仍逐字保留为附录**

---

## 五、边界与注意

- **3D / 2D 姿态均为可视化示意**（连杆长度固定为示意值，J1 绕世界 Z、J2/J3 局部 Y 共面、J4 绕连杆轴），非精确运动学 / DH 解算，不可用于轨迹规划或碰撞检测。
- 关节限位只做**显示与提示**，不拦截真机指令；实际限位由固件负责。
- 本项目对 P340 **只读**：不调用 `set_angles()` / `go_zero()` / 夹爪与激光接口，避免误动作。
- 3 轴模式若真机返回 4 个角度，面板按前 3 个显示；4 轴模式若真机只返回 3 个，J4 显示 `--`。
- `get_coords_info()` 返回值长度随配置变化（X/Y/Z 或 X/Y/Z/E），面板按实际长度填写，缺省显示 `--`。
- 六轴示例 `01_led_blink.py` / `02_read_angles.py` / `03_gohome.py` 依赖 `set_color()` / `get_angles()` 等六轴 API，**不适用于 P340**。

## 六、参考链接

- 产品页：<https://www.elephantrobotics.com/ultraarm-p340/cn>
- 技术参数页：<https://www.elephantrobotics.com/ultraarm-p340-spcn/>
- 中文文档首页：<https://docs.elephantrobotics.com/docs/ultraArm/>
- Python API：<https://docs.elephantrobotics.com/docs/ultraArm/3-HowToUseultraArm/2-SoftwareControl/4-Python/2-PythonAPI.html>
- 关节控制：<https://docs.elephantrobotics.com/docs/ultraArm/3-HowToUseultraArm/2-SoftwareControl/4-Python/3-JointControl.html>
