# decision 模块说明

## 一句话概要
`decision` 模块负责车载/机器人高层决策与设备管理：设备（特别是 Modbus 设备）管理、任务分发与执行、示教（路径）管理与跟随、与上层/下层的消息接口（消息/服务/动作）等。

## 功能清单（高层）
- Modbus 设备管理：通过 `src/DeviceManager.*` 系列文件对 Modbus 从机进行读取/写入、心跳与异常处理。
- 设备节点包装：`DeviceManagerNode` 以 ROS 节点形式运行设备管理逻辑，负责与 ROS 通信（topic/service/param）。
- 任务与执行框架：使用 `action/BehaviorExecuter.action`、`action/TaskExecuter.action`、`msg/TaskCmd.msg`、`srv/TaskService.srv` 实现任务下发、执行控制与状态反馈。
- 示教路径管理与跟随：`path/`、`route/` 目录保存示教点与路径（XML/文本格式），决策模块提供加载/选择/执行示教路径的能力，并与导航栈或运动控制器对接。
- 可视化与调试：`rviz_cfg/`、`launch/display.launch` 提供 RViz 配置与可视化启动文件。
- 辅助：`scripts/`（如 `pub_start_lio.py`）用于调试或启动序列；`sounds/` 存放提示音。

## 关键文件说明
- `launch/decision.launch`：启动决策相关节点的主 launch（启动参数通常会放在此处）。
- `src/DeviceManager.cpp`、`src/DeviceManager.hpp`：Modbus 设备管理核心实现（打开端口、读写寄存器、超时/重连、状态上报）。
- `src/DeviceManagerNode.cpp`：把 DeviceManager 包装为 ROS 节点，连接参数服务器、topic/service、以及必要的周期任务。
- `src/MqttInterface.hpp`：MQTT 相关接口（如果系统用到远程消息或云端上报）。
- `src/cJSON.*`：用于解析/生成 JSON 配置或消息（配置文件、网络消息等）。
- `msg/TaskCmd.msg`：任务命令消息定义（task id、参数、目标点等）。
- `srv/TaskService.srv`：任务服务接口。
- `action/`：行为与任务执行的 action 定义，可用于异步任务控制。
- `path/`、`route/`：示教点、路径与路线文件（XML），为示教路径跟随提供数据源。

## 使用说明（快速入门）
下面给出通用、可移植的使用步骤与示例参数（注意：某些 topic / service / 可执行文件名按仓库约定推断，实际请参照源码或 `decision.launch` 中的命名）：

1. 编译与环境
   - 进入工作空间（假设为 `catkin` 工作区）：
     ```bash
     cd ~/catkin_ws  # 或你的工作空间
     catkin_make
     source devel/setup.bash
     ```

2. 配置参数
   - 在 `launch/decision.launch` 中或通过 `rosparam` 设置 Modbus 相关参数（串口、波特率、从机地址等）。示例（占位）：
     ```bash
     rosparam set /decision/modbus/port /dev/ttyUSB0
     rosparam set /decision/modbus/baud 115200
     rosparam set /decision/modbus/type serial  # 或 tcp
     ```

3. 启动
   - 启动决策模块（主 launch）：
     ```bash
     roslaunch decision decision.launch
     ```
   - 可选：在另一终端启动显示/rviz：
     ```bash
     roslaunch decision display.launch
     ```

4. 任务下发与示教路径执行（示例）
   - 使用消息/服务/动作下发任务：
     - 通过 topic 发布 `TaskCmd`：用于简单即时任务
     - 通过 action client 调用 `TaskExecuter` / `BehaviorExecuter`：用于带进度与取消的异步任务
     - 通过 `TaskService.srv` 的 service 调用请求执行具体任务

   - 路径跟随示例（伪命令，替换为实际的 service/action 名称）：
     ```bash
     rosservice call /decision/task_service "task_name: 'follow_route' route_file: 'route/2F1A.xml'"
     # 或者通过 action client 发送目标，action 名称与定义请参考 action/*.action
     ```

## Modbus（设备管理）详解
实现要点与使用方法：

- 责任
  - 负责与现场 Modbus 从机建立通信（串口/RTU 或 TCP），周期性读取关键寄存器、写入控制寄存器、处理异常（超时/CRC/重连）。
  - 将设备状态以 ROS topic 或 service 的形式上报给系统。也可能通过 `cJSON` 格式进行配置/上报。

- 关键实现点（在 `DeviceManager.*`）
  - 串口/TCP 打开、配置（端口、波特率、数据位、校验、停止位）
  - 周期读取（如读保持寄存器）和事件驱动写入
  - 错误重试、重连逻辑和设备离线检测
  - 将原始寄存器翻译为语义化字段（例如：开关、故障码、电量、传感器读数等）并发布为 ROS 消息

- 配置方式（推荐）
  - 把设备地址、寄存器映射、读写周期放到 ROS 参数或 JSON 配置文件中（仓库含 `cJSON`，故很可能支持 JSON 配置）。

- 使用示例（占位，实际 Topic/Service 名称请查看 `DeviceManagerNode.cpp`）：
  - 查询状态（伪例）：
    ```bash
    rosservice call /decision/device_manager/get_state "device_id: 1"
    ```
  - 写寄存器（伪例）：
    ```bash
    rosservice call /decision/device_manager/write_register "device_id: 1 register: 10 value: 1"
    ```

- 常见问题与调试
  - 串口权限：确保运行节点的用户对 `/dev/ttyUSB*` 有读写权限（使用 `sudo usermod -a -G dialout $USER` 并重新登录）。
  - 波特率 / 数据格式错误会导致 CRC 校验失败或超时。先用 `modbus-cli` 或 `modpoll` 在终端验证设备是否能连通。

## 示教路径（Teach）与跟随详解
- 存放位置
  - 示教点/路径位于 `path/`（可能按名字分目录保存点集），路线文件位于 `route/`（XML 格式），这些文件是示教轨迹或导航路线的持久化表示。

- 数据格式
  - 仓库使用 XML（或自定义格式）保存一条路线的一组路径点（包括坐标、朝向、触发事件、停留时间等），具体字段请参阅 `route/*.xml` 示例。

- 执行流程（典型）
  1. 在地面站或通过工具把示教点保存到 `path/`，并生成/选择一个 `route/*.xml`。
  2. 在决策模块通过服务或 action 请求载入该 route 文件。
  3. 决策模块将路线拆成若干导航目标，依次发送到导航栈（如 `move_base`）或底层运动控制节点。
  4. 每个目标到达后，依据 route 中的指令执行附带动作（例如：播放提示音、操作 Modbus 输出、等待若干秒、拍照等）。
  5. 路线执行完毕后，上报任务完成状态。

- 常见触发方式
  - 手动下发：通过 `rosservice` / topic 发布一条任务命令（TaskCmd）来执行某个 route
  - 自动触发：根据条件（定时、传感器触发、远端指令）自动切换到执行状态

- 调试技巧
  - 在仿真或 RViz 中先用 `display.launch` 可视化 route 与当前位姿，确认坐标系一致。
  - 将路线拆短并单步执行，确认每个导航目标在 move_base 能够成功寻路。

## 小合同（inputs/outputs / 错误模式 / 成功标准）
- 输入：ROS 参数（modbus 串口/配置）、`route/*.xml` 路径文件、通过 topic/service/action 下发的 `TaskCmd` 或 action goal。
- 输出：设备状态 topic、任务状态 topic/service/action feedback、对导航/运动节点的 goal/控制命令。
- 错误模式：Modbus 超时/CRC 错误、路径不可达（导航失败）、配置错误（坐标系不一致）、权限问题（串口）。
- 成功判定：Modbus 设备读写稳定（重连/错误率低）、示教路线可在仿真或实车上完整执行并依次完成路点动作。

## 常见边界情况与建议（3-5 条）
- 网口/串口权限或占用：确保设备节点独占串口或有重连策略。
- 地图与路线坐标系不一致：在 RViz 中确认 `map`/`odom`/`base_link` 之间的 TF 是否正确。
- 导航无法规划：检查障碍物、costmap、局部规划器参数，或将路线分段并在低速下测试。
- Modbus 大量 IO 阻塞：设置合理的轮询间隔，避免在读写高频寄存器时阻塞主循环。

## 未来扩展建议（可选）
- 增加 JSON/YAML 配置文件的热加载能力，便于现场快速调整寄存器映射与轮询频率。
- 为示教路径提供图形化编辑器（导入/导出 XML），并把 action 状态机的日志化与回放能力完善。

---

注意：上述示例命令与某些 topic/service 名称为占位/推断，具体名称请参考源码（如 `src/DeviceManagerNode.cpp`、`launch/decision.launch`、`action/*.action`）以获得精确接口。若需要，我可以：

- 1) 解析并生成一份精准的 API 列表（从源码中抽取所有 topic/service/action 名称与消息字段）；或
- 2) 直接把本文件合并到仓库的 `readme.md` 或覆盖现有说明（按你的选择）。
