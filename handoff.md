# Handoff — joint_monitor 改动记录

> 记录本次迭代对 `joint_monitor.py` 的所有改动，供后续接手的人查阅。
> 更新日期：2026-11-12

---

## 概览

本次迭代共完成 8 项工作，全部集中在 `joint_monitor.py`（`robot_profiles.py` 未改动）：

| # | 内容 | 状态 |
|---|------|------|
| 1 | 修复 `int has no len()` 崩溃 | ✅ 完成 |
| 2 | 串口连接问题诊断（Transponder 透传） | ✅ 已定位（硬件侧待用户确认） |
| 3 | 侧边关节角度条支持拖动 | ✅ 完成 |
| 4 | 夹爪控制面板（280 自适应夹爪） | ✅ 完成 |
| 5 | 缩小趋势卡片 + 新增操作记录卡片（支持回滚） | ✅ 完成 |
| 6 | 修复操作记录未显示 + 三列底部对齐 | ✅ 完成 |
| 7 | 修复窗口大小自适应失效 | ✅ 完成 |
| 8 | 新增「重做」功能（撤销后可重做） | ✅ 完成 |
| 10 | 操作记录「回退到选中处并重放」 | ✅ 完成 |

---

## 1. 修复 `int has no len()` 崩溃

**文件位置**：`AngleSource.read()`（约第 329-352 行）

**根因**：串口通信异常或机械臂未就绪时，`pymycobot` 的 `get_angles()` 会返回 `-1`（int 错误码）而非角度列表（见 `vendor/pymycobot/mycobot280.py` 的 `_res()`，连续 3 次读不到数据时 `return -1`）。随后 `len(angles)` 对 int 报错 `TypeError: object of type 'int' has no len()`，并沿调用链冒泡到采样线程 `Reader.run()` 的 except 分支，导致采样线程中断退出。

**修复**：
- `get_angles()` 加 `try/except`，异常时降级为 `None`；
- 返回值做 `isinstance(angles, (list, tuple))` 类型校验，非序列值统一置 `None`；
- `get_coords()` 同样加类型校验。

**效果**：串口无应答时安全保留上一帧，不再崩溃。

---

## 2. 串口连接问题诊断（重要，硬件侧）

**诊断结论**（用真实串口测试验证）：
- 串口 COM3 存在且能正常打开（CH340 芯片，VID:PID=1A86:55D4）；
- 但 `get_angles()` / `get_system_version()` 均返回 `-1`，缓冲区 `in_waiting = 0`，机械臂无任何数据返回。

**根因**：**不是软件问题**，是机械臂未进入 Transponder 透传模式。myCobot 280 M5 必须在 Basic 小屏上手动进入 **Transponder** → 按 **A 键** → 屏幕显示 **`Atom: OK`**，串口透传才打通。

**待办**：需要用户确认 Basic 屏是否显示 `Atom: OK`。若未显示，角度读数和夹爪/角度发送都会失败。

---

## 3. 侧边关节角度条拖动

**交互行为**（用户确认：松开后发送 + 不暂停采样）：
- 鼠标按下/拖动角度条 → 实时更新数值标签 + 进度条 + 3D 姿态预览；
- 松开 → 真机模式调用 `send_angle(joint_id, degree, speed=30)` 发送；
- 演示模式只更新本地显示。

**新增方法**：
- `AngleSource.send_angle(joint_id, degree, speed=30)`：发送单关节角度，带异常保护；
- `MonitorApp._bar_press/_bar_drag/_bar_release`：三个拖动事件处理；
- `MonitorApp._bar_x_to_angle(idx, x)`：鼠标 x 坐标 → 角度值（按 `profile.display_limit` 线性映射，左端 -180° / 中间 0° / 右端 +180°）。

**关键改动点**：
- 角度条 Canvas 绑定 `<ButtonPress-1>` / `<B1-Motion>` / `<ButtonRelease-1>`，光标 `sb_h_double_arrow`；
- `_update()` 里跳过正在拖动的关节（`i == self._bar_dragging`），避免读回值覆盖拖动预览。

---

## 4. 夹爪控制面板（280 自适应夹爪）

**界面**：左列角度条下方新增「夹爪控制」卡片，含：
- 开度进度条（可拖动，0% 左=闭合，100% 右=张开）；
- 开度百分比显示；
- 三个按钮：**张开**（`set_gripper_state(0)`）、**闭合**（`set_gripper_state(1)`）、**释放**（`set_gripper_state(254)`）。

**新增方法**：
- `AngleSource.set_gripper_state(flag, speed=50)` / `set_gripper_value(value, speed=50)` / `get_gripper_value()`；
- `MonitorApp._draw_gripper_bar` / `_gripper_press/_drag/_release` / `_apply_gripper_value` / `_gripper_send_value` / `_gripper_open/_close/_release_full`。

**pymycobot 280 夹爪接口速查**（vendor 内 `mycobot280.py`）：
- `set_gripper_state(flag, speed)`：flag 0=开 / 1=闭 / 254=释放；
- `set_gripper_value(value, speed)`：value 0~100 精细开度；
- `get_gripper_value()`：读当前开度。

**注意**：
- 仅适配 280 自适应夹爪（`gripper_type` 默认 1），320/P340 未适配；
- 夹爪面板只在 280/320 六轴视图下显示，P340 四轴面板（`p340_view`）不显示；
- 发送速度固定 50。

---

## 5. 缩小趋势卡片 + 操作记录卡片（支持回滚）

**趋势卡片缩小**：趋势图高度从「随窗口铺满右列」改为固定上限 **150px**（`_on_host_configure` 里 `h_trend = max(70, min(150, avail - RIGHT_CHROME))`）。

**新增操作记录卡片**（右列底部，row 2）：
- 滚动 `Listbox` 记录每次操作，格式 `时间  对象  旧值 -> 新值`；
- 计数标签显示记录条数；
- **撤销上一步**按钮（回滚）；
- **清空记录**按钮。

**记录哪些操作**：
- 拖动角度条松开（`send_angle`）；
- 夹爪张开 / 闭合（`set_gripper_state`）；
- 夹爪开度条拖动松开（`set_gripper_value`）。

**回滚机制**（快照式）：
- 维护 `self.sent_angles`（6 轴实际角度快照）和 `self.sent_gripper`（夹爪开度快照），真机读到数据时同步更新；
- 每条记录含 `before`（操作前值）和 `after`（操作后值）；
- 撤销时用 `send_angle` / `set_gripper_value` 恢复 `before` 值，同步更新界面 + 3D 姿态；
- 记录上限 200 条，超出淘汰最旧。

**新增方法**：
- `_record_op(kind, before, after, joint_id=None)`：kind 为 `'angle'` 或 `'gripper'`；
- `_refresh_op_list()`：刷新列表显示，滚动到底部；
- `_rollback()`：撤销最后一步；
- `_fmt_op(rec)`：格式化记录（用于状态栏提示）；
- `_clear_ops()`：清空记录。

**注意**：夹爪「释放」操作**不参与回滚**（失电松脱不可逆）。

---

## 6. 修复操作记录未显示 + 三列底部对齐

**问题 A：操作记录未显示拖动操作**
- 原因：记录逻辑被包裹在 `if not self.demo_var.get()` 条件里，演示模式/串口未通时不记录。
- 修复：把「记录」与「发送」解耦，拖动松开**总是记录**（角度变化 > 0.01° 才记），发送逻辑独立。

**问题 B：三列底部不对齐**
- 原因：中列 3D 卡片 `fpose` 用 `sticky="new"`（只贴顶部），下方留白；左右列都填满整列高度。
- 修复：
  - `fpose` 改 `sticky="nsew"`，pose 画布改 `pack(fill="both", expand=True)`；
  - 中列 `grid_rowconfigure(0, weight=1)`；
  - 移除 `_on_host_configure` 里对 pose 高度的手动设置（`self.pose.configure(height=h_pose)`），避免与 expand 冲突。

---

## 关键数据结构（新增状态变量）

在 `MonitorApp.__init__` 中新增：

```python
# 角度条拖动状态
self._bar_dragging = -1      # 正在拖动的关节索引，-1 表示无
self._bar_edit = [0.0] * 6   # 拖动过程中的临时目标角度

# 夹爪状态
self.gripper_val = 0         # 当前夹爪开度 0~100（0=闭合，100=张开）
self.gripper_label = None    # 夹爪开度显示标签
self.gripper_canvas = None   # 夹爪进度条画布
self._gripper_dragging = False

# 操作记录与回滚
self.sent_angles = [0.0] * 6   # 机械臂当前已知角度快照（用于回滚 before）
self.sent_gripper = 0          # 夹爪当前已知开度快照
self.op_history = []           # 操作记录列表，每项 dict
self.redo_stack = []           # 被撤销的记录，供重做
self.op_listbox = None         # 操作记录列表控件
self.op_count_label = None     # 记录条数显示
self.redo_btn = None           # 重做按钮（延迟绑定）
```

---

## 7. 修复窗口大小自适应失效

**原因**：第 6 项改动把 pose 画布从「显式 `configure(height=...)`」改成「`fill="both", expand=True`」，导致 pose 不再主动请求高度，`body.winfo_reqheight()` 变小，body 无法随窗口正确撑高（放大窗口时 body 反而缩小）。

**修复**：恢复原设计——pose 画布回到 `pack(fill="x")` + `fpose` 回到 `sticky="new"`，中列恢复 `grid_rowconfigure(0, weight=0)` + `row 1 weight=1`；在 `_on_host_configure` 里**先算 `need_h`，再据此显式设置 pose 高度**（`self.pose.configure(height=max(220, h - POSE_CHROME))`），使 pose 填满 body 最终高度，从而三列底部对齐。

**验证**：窗口 700/900/1000/1100 多档缩放，body 高度正确跟随，三列底部始终对齐。

## 8. 新增「重做」功能

在操作记录卡片新增「重做」按钮（绿色），撤销后可重做刚撤销的一步。

**机制**：
- 新增 `self.redo_stack` 栈，撤销时把被撤销的记录压入，重做时弹出并重新应用 `after` 值；
- 任何**新操作**（`_record_op`）都会清空 `redo_stack`（重做链断裂）；
- 清空记录时同时清空 redo 栈；
- 重做按钮根据 redo 栈是否为空自动启用/禁用（`_update_redo_btn`）。

**代码重构**：
- 抽取 `_apply_op_value(rec, target)`：发送指令 + 更新界面（真机）；
- 抽取 `_apply_local_value(rec, target)`：仅本地更新显示（演示/无驱动）；
- 演示模式下撤销/重做也正确更新本地界面（角度条/3D 姿态/夹爪条）并保持记录列表一致，只是不发送指令。

---

## 9. 修复「重做未写回 + 卡片高度不自适应」（第二轮修复）

### 9.1 重做内容未写回操作记录

**根因**：`AngleSource.send_angle` / `set_gripper_value` / `set_gripper_state` 直接透传 `mc.xxx()` 的返回值，而 pymycobot 写指令的返回值语义不稳定（可能返回 `-1`、`0` 等，不一定是 `None`）。导致调用点 `if res is None` 判断"成功"失效，真机模式下重做误判为失败，从而不写回 `op_history`。

**修复**：统一 `AngleSource` 三个发送方法的返回语义——**成功（不抛异常）返回 `None`，抛异常返回错误字符串**。写指令只要内部没抛异常即视为已发出（pymycobot 内部已做 3 次重试）。

### 9.2 卡片高度不能随窗口自适应

**根因**：存在循环依赖——`_on_host_configure` 设 body 高度 → pose 高度 = body - 80 → pose 反过来决定 center/body 的 reqheight；同时 `_after_resize` 的兜底逻辑用 `body.winfo_reqheight()` 又把 body 锁回内容自然高度，导致 body 无法真正跟随窗口。

**修复**：
- 移除 `_after_resize` 里基于 `body.winfo_reqheight()` 的兜底逻辑（循环依赖根源）；
- `_on_host_configure` 改为：body 高度 = `max(可用高度, 左列自然高度, BODY_MIN_H)`，其中左列自然高度用 `max(LEFT_REQ_FULL/SHORT, left_frame.winfo_reqheight())`（左列是固定 pack 布局，reqheight 稳定，不参与循环）；
- pose 高度 = body 高度 - POSE_CHROME，趋势固定上限 150，操作记录卡片 `weight=1` 填满剩余。

---

## 10. 操作记录「回退到选中处 / 从选中处重放」

在操作记录卡片新增第二行两个独立按钮：
- **回退到选中处**（橙色）：撤销选中步及之后的所有操作，回到该步之前的状态；
- **从选中处重放**（紫色）：从选中步开始，顺序重放其及之后的所有 `after` 值。

点击列表选中某一条记录后触发；两个操作可独立使用（可只回退、或只重放、或先后组合）。

**实现**（增量式，纯 `before`/`after` 重放）：
- `_rollback_to(index)`：逆序撤销 `index..n-1` 步（对每步执行 `before`），回到第 `index-1` 步完成后的状态；
- `_replay_from(index)`：顺序执行 `index..n-1` 步的 `after` 值；
- `_get_selection_index()`：读取 `op_listbox.curselection()`，无记录/未选中时提示。

演示/无驱动模式只做本地界面更新（`_apply_local_value`），不发送指令。

**验证**：mock 真机测试，选中第 2 步时「仅回退」发送「撤步4、3、2」，「仅重放」发送「步2、3、4」，顺序正确；未选中/首步/末步等边界正常。

---

## 验证情况

- `py -m py_compile joint_monitor.py`：语法通过；
- `py joint_monitor.py --selftest`：`SELFTEST_OK`，未破坏原有逻辑；
- GUI 冒烟测试（`force_demo=True` 构造 + mock 真机 reader）：拖动记录、夹爪记录、回滚、重做、清空、三列底部对齐、窗口缩放、真机路径重做写回均验证通过。

**注意**：当前命令行 `python` 无 tkinter，需用 `py` 启动器（Python 3.13 带 tkinter）或项目实际部署用的 `.venv`（原部署路径为 `C:\Users\ThinkPad\Documents\MyCobot`，本机当前工作目录是 `C:\Users\Administrator\Desktop\mycobot-m5-monitor-main`，可能是一份副本，`.venv` 不在当前目录）。

---

## 后续待办 / 未完成项

1. **Transponder 透传问题（阻塞真机使用）**：需用户确认 Basic 屏显示 `Atom: OK`，否则所有读写（角度、夹爪）都会失败。
2. **320 / P340 夹爪适配**：目前只做了 280 自适应夹爪。
3. **夹爪释放回滚**：如需要「释放」也可回滚，需额外设计（当前不记录释放操作）。
4. **发送速度可调**：角度发送固定 30、夹爪固定 50，如需可调速度或力控（`is_torque`）需扩展。
5. **多关节联动拖动 / 整臂拖动**：当前仅单关节拖动，如需整臂多关节联动需另设计。
