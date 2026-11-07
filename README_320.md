# myCobot 320 M5 适配说明（增量，不改写 280 版本）

本项目原本只支持 **myCobot 280 M5**。本次按[官方产品参数页](https://www.elephantrobotics.com/mycobot-320-m5-cn/parameter/)与官方文档**新增** 320 M5 适配，采取「默认型号不变 + 显式切换」的方式：

- **不带 `--model` 运行时，行为与适配前完全一致（280 M5）**；
- 加 `--model 320` 或在界面右下角「型号」下拉切到 `320`，即按 320 M5 的驱动、限位、量程、几何比例与硬件提示运行；
- `README.md` 现已在开头新增三型号总览，**原 280 全文一字未改地保留为附录章节**；`requirements.txt`、`setup.ps1`、`start_monitor.cmd`、`vendor/` 未改动。

---

## 一、参数对照

| 项目 | 280 M5（原有） | 320 M5（新增） | 320 数据来源 |
|---|---|---|---|
| Python 驱动类 | `pymycobot.mycobot280.MyCobot280` | **`pymycobot.mycobot320.MyCobot320`** | 官方 Python API 页 |
| 波特率 | 115200 | **115200** | 官方示例 `MyCobot320('COM3', 115200)` |
| 自由度 | 6 | **6** | 产品参数页 |
| 最大工作半径 | 280 mm | **350 mm** | 产品参数页 |
| 最大负载 | 原项目未声明 | **1000 g（1 kg）** | 产品参数页 |
| 净重 | 原项目未声明 | **3360 g**（文档机规格页记 3.3 kg） | 产品参数页 |
| 重复定位精度 | 原项目未声明 | **±0.5 mm** | 产品参数页 |
| 关节运动范围 | 界面沿用 ±180 显示量程 | **J1~J5 −165~+165°，J6 −175~+175°** | 文档 2.1.1 机器规格参数 |
| 供电 | 12V / 3–5A，DC 5.5×2.1 | **DC 24V / 120W**（机规格页记 24V 9.2A），KPPX-4P 4 针插座 | 产品参数页 / 2.1.1 / 2.1.4 |
| 主控 / 显示 | M5Stack-Basic + Atom | **ESP32 主控 + ESP32×2 辅控，2.0″ 320×240 IPS** | 产品参数页 |
| 通信 | 串口（Transponder 转发） | 串口 / Type-C / 蓝牙 / TCP/IP / MODBUS | 产品参数页 |
| 寿命 / 工作温度 | — | 2000 h / 0~45 ℃ | 产品参数页 / 2.1.1 |

> **电源数值以随机适配器铭牌为准**：官网参数页写 `DC 24V 120W`，官方文档机规格页写 `24V 9.2A`，两处不一致，本适配未擅自取舍，已在 `robot_profiles.py` 中同时标注。**切勿给 320 M5 接 12V 电源，也不要套用 280 的 Atom 固件 / `Atom: OK` 流程。**

---

## 二、适配了什么

| 位置 | 280 行为 | 320 行为 |
|---|---|---|
| 真机驱动 | `MyCobot280(port, baud)` | `MyCobot320(port, 115200)`，由 `profiles.create_robot()` 懒加载 |
| 显示量程 | ±180，趋势刻度 ±180/±90/0 | ±175，刻度 ±175/±87.5/0 |
| 限位检查 | 不做（保持原样） | 逐关节按官方限位判定，超限在状态栏提示 `超限位 Jx` |
| 3D 姿态 | 原连杆长度 | 连杆 / 基座高度 / 地面网格按 **350/280 = 1.25** 缩放，仍为几何示意 |
| 界面文案 | 原 `TIP_FULL/TIP_SHORT` | 320 专属说明（24V、限位、比例、固件流程） |
| 无响应提示 | `检查 Transponder / 供电 / 波特率` | `检查 串口转发 / 供电 / 115200 波特率` |
| 状态栏 | — | 新增「型号」下拉，可随时切换（采集中禁止切换，防驱动错配） |
| 自检 | 原有检查 | 追加逐型号驱动可导入检查 + 320 几何/限位断言 |

**依赖零变化**：`pymycobot==3.9.9` 已同时内置 `mycobot280.py` 与 `mycobot320.py`，`vendor/` 里两个模块都在，无需升级或新增 wheel。

---

## 三、使用方法

```powershell
# 默认：280 M5，与适配前完全一致
.\.venv\Scripts\python.exe check_env.py
.\.venv\Scripts\python.exe joint_monitor.py

# 新增：320 M5
.\.venv\Scripts\python.exe check_env.py --model 320
.\.venv\Scripts\python.exe check_env.py --model 320 --port COM3
.\.venv\Scripts\python.exe joint_monitor.py --model 320
.\.venv\Scripts\python.exe joint_monitor.py --model 320 --demo

# 示例（三个均支持 --model）
.\.venv\Scripts\python.exe examples\01_led_blink.py --model 320 --dry-run
.\.venv\Scripts\python.exe examples\02_read_angles.py --model 320 --port COM3
.\.venv\Scripts\python.exe examples\03_gohome.py --model 320 --port COM3 --yes

# 无界面自检（含 320 适配层断言，不打开串口）
.\.venv\Scripts\python.exe joint_monitor.py --selftest
```

界面方式：启动后在**右下角状态栏**的「型号」下拉选择 `280` / `320`，标题、说明、量程、趋势刻度与 3D 几何会同步刷新。若正在采集，需先点「停止采集」。

**320 M5 硬件前置**：接 DC 24V 电源 → Type-C 连接 M5Stack-Basic → 按[官方 320 M5 首次使用流程](https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/4-FirstInstallAndUse/4.2-M5/4.2_320_M5_firstUse.html)安装驱动、烧录固件并确认串口转发就绪 → 选 COM 口开始采集。

---

## 四、文件清单

**新增**
- `robot_profiles.py`：型号参数表、驱动工厂、限位判定、几何缩放、文案（唯一适配真源）
- `README_320.md`：本说明
- `backup_280_original/`：改动前的 `joint_monitor.py`、`check_env.py`、`examples/*.py` 原件备份

**接入（保留原有逻辑，仅增加切换点）**
- `joint_monitor.py`：`--model`、型号下拉、按型号驱动/量程/几何/文案/限位提示、`--selftest` 追加 320 断言
- `check_env.py`：`--model`、逐型号驱动检查、按型号显示供电与固件提示
- `examples/01_led_blink.py`、`02_read_angles.py`、`03_gohome.py`：`--model`、按型号创建驱动

**未改动**：`requirements.txt`、`bootstrap.py`、`setup.ps1`、`run_via_task.ps1`、`start_monitor.cmd`、`vendor/`（`README.md` 仅新增双型号总览，原 280 全文保留为附录）。

回滚：适配改动已提交为 `00a1a84`，280 原版快照见 tag `v1.0.0`。用 `git revert 00a1a84` 回退，或 `git checkout v1.0.0 -- joint_monitor.py check_env.py examples/` 恢复单文件；`backup_280_original/` 亦保留原件副本。

---

## 五、边界与后续

- 3D 姿态仍是**串联链几何示意**，按工作半径等比缩放连杆，**不是精确 DH 参数解算**，不可用于碰撞检测或轨迹规划；320 的真实 DH/URDF 参数未在本次适配中引入。
- 320 的 `MyCobot320` 与 280 共用 `get_angles()` / `get_coords()` / `_serial_port` 接口，已核对 `vendor/pymycobot 3.9.9` 源码；`send_angles()` / `send_angle()` / `set_color()` 来自同一 `CommandGenerator` 基类，示例可直接复用。
- 关节限位用于**显示与提示**，不拦截真机指令；实际限位仍由固件与 `pymycobot` 的参数校验负责。
- 若后续要精确运动学或笛卡尔坐标校验，需引入 320 官方 URDF/DH 表，另开一轮。

## 六、参考链接

- 产品参数页：<https://www.elephantrobotics.com/mycobot-320-m5-cn/parameter/>
- 320 M5 中文文档首页：<https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/>
- 机器规格参数（含关节范围）：<https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/2-ProductFeature/2.1_320_M5_product/2.1.1-MachineSpecification.html>
- 电气特性参数：<https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/2-ProductFeature/2.1_320_M5_product/2.1.4-ElectricalCharacteristicParameter.html>
- Python API：<https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/10-ApplicationBasePython/10.2_320_M5-ApplicationPython/2_API.html>
- 首次使用：<https://docs.elephantrobotics.com/docs/mycobot_320_m5_cn/4-FirstInstallAndUse/4.2-M5/4.2_320_M5_firstUse.html>
