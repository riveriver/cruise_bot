# decision 模块 — API 参考

本文件汇总 `decision` 模块中从源码提取的 ROS 接口（topic、service、action、自定义消息及字段）。所有名称与字段均来自模块源码（`src/DeviceManager.*`、`action/*.action`、`msg/*.msg`、`srv/*.srv`），供开发与集成参考。

## 目录
- Topics（发布/订阅）
- Services（服务）
- Actions（action 定义与常用 server 名称）
- 自定义消息（字段）
- 外部引用消息类型
- 运行时关键名称汇总

---

## Topics
说明格式：topic — 方向 — ROS 消息类型 — 备注

发布（Publish）
- `cmd_vel` — 发布 — `geometry_msgs/Twist` — 手动/遥控速度输出
- `record_path` — 发布 — `nav_msgs/Path` — 记录/示教路径
- `record_zone` — 发布 — `geometry_msgs/PolygonStamped` — 记录区域/多边形
- `/move_base_simple/goal` — 发布 — `geometry_msgs/PoseStamped` — 简单导航目标
- `current_pose` — 发布 — `geometry_msgs/PoseStamped` — 当前目标/位姿

订阅（Subscribe）
- `/battery_state` — 订阅 — `ranger_msgs::BatteryState` — 电池状态输入
- `/sensors/air_data` — 订阅 — `cruise_decision::AirSensors` — 空气传感器数据
- `/sensors/air_result` — 订阅 — `cruise_decision::AirSensors` — 空气质量结果
- `/base_pose_ground_truth` — 订阅 — `nav_msgs::Odometry` — 真实/参考位姿
- `/clicked_point` — 订阅 — `geometry_msgs::PointStamped` — RViz 点击点
- `/move_base_flex/global_costmap/costmap` — 订阅 — `nav_msgs::OccupancyGrid` — 全局 costmap
- `/move_base_flex/global_costmap/costmap_updates` — 订阅 — `map_msgs::OccupancyGridUpdate` — costmap 更新

> 备注：源码中头文件声明了 `traffic_goal_sub_`，但未在 `init()` 中显式指定 topic 名称（可能由参数/launch 指定）。

---

## Services
- Service Server:
  - `task_cmd` — 类型 `cruise_decision::TaskService`
    - Request 字段：
      - uint8 type         # 可选值示例：EXECUTE=0, RECORD=1, LOAD_TRAFFIC_ROUTE=2, QR_NAV=3
      - uint8 command      # 可选值示例：START=0, PAUSE=1, STOP=2, KEEP_TEACH=1, ...
      - string dir
      - string path_name
      - string map
    - Response 字段：
      - string message

- Service Clients（节点作为 client 使用，部分由源码注释或头文件声明）
  - `wakeup_theta_client_` — (示例/注释) 目标 service 名称可能为 `wakeup_theza`（请以运行时/launch 为准）
  - `start_theta_client_` — (示例/注释) 目标 service 名称可能为 `start_theza`

> 备注：部分 service 名称可能由 launch 或参数动态指定，需以实际运行的 launch/参数为准。

---

## Actions
自定义 action 定义位于 `action/`。

- `TaskExecuter.action` (`cruise_decision::TaskExecuterAction`)
  - Goal:
    - string id
    - string task_type
    - string task_time
    - string task_location
    - string task_project
  - Result:
    - float32 progress
    - uint32 code
    - string msg
  - Feedback:
    - float32 progress

- `BehaviorExecuter.action` (`cruise_decision::BehaviorExecuterAction`)
  - Goal:
    - string type
    - string cmd
    - string id
    - string time
    - string pose
  - Result:
    - string code
    - string message
  - Feedback:
    - float32 progress

DeviceManager 中构造/等待/使用的 action client 名称（关键依赖）：
- `move_base_flex/move_base` — action type `mbf_msgs::MoveBaseAction`（GotoCtrl）
- `move_base_flex/exe_path` — action type `mbf_msgs::ExePathAction`（ExeCtrl）
- `task_executer/building_service` — `cruise_decision::TaskExecuterAction`（building_service）
- `task_executer/air_quality` — `cruise_decision::TaskExecuterAction`（air_quality）
- `task_executer/lio_relocate` — `cruise_decision::TaskExecuterAction`（lio_relocate）

---

## 自定义消息（msg）与字段
- `cruise_decision/TaskCmd` (`msg/TaskCmd.msg`)
  - string id
  - int64 time
  - string pose
  - string cmd

- `cruise_decision/AirSensors` (`msg/AirSensors.msg`)
  - std_msgs/Header header
    - time stamp
  - float32 temperature
  - float32 humidity
  - uint32 pm1_0
  - uint32 pm2_5
  - uint32 pm10
  - float32 tvoc
  - uint32 co2
  - float32 formaldehyde
  - float32 light1
  - float32 light2
  - uint32 noise
  - float32 ozone_concentration
  - float32 airflow
  - float32 co
  - float32 rn
  - float32 no2

- `cruise_decision/TaskService`（srv/TaskService.srv）
  - 文件顶部包含枚举常量（EXECUTE/RECORD/...、START/PAUSE/STOP/... 等），请求与响应字段见 Services 部分。

---

## 外部/第三方消息类型（源码引用）
- geometry_msgs::Twist, PoseStamped, PointStamped, PolygonStamped
- nav_msgs::Path, Odometry, OccupancyGrid
- map_msgs::OccupancyGridUpdate
- mbf_msgs::MoveBaseAction, mbf_msgs::ExePathAction
- move_base_msgs::MoveBaseAction
- ranger_msgs/*（BatteryState, ActuatorState, DriverState, MotorState, MotionState, SystemState, TriggerParkMode）
- std_msgs/String, std_msgs/Bool

---

## 运行时关键名称汇总（便捷查阅）
- Topics publish: `cmd_vel`, `record_path`, `record_zone`, `/move_base_simple/goal`, `current_pose`
- Topics subscribe: `/battery_state`, `/sensors/air_data`, `/sensors/air_result`, `/base_pose_ground_truth`, `/clicked_point`, `/move_base_flex/global_costmap/costmap`, `/move_base_flex/global_costmap/costmap_updates`
- Service: `task_cmd` (`cruise_decision::TaskService`)
- Actions: `move_base_flex/move_base`, `move_base_flex/exe_path`, `task_executer/building_service`, `task_executer/air_quality`, `task_executer/lio_relocate`

---

## 校验与注意事项
- 本清单基于仓库内 `decision` 模块的源码静态提取；部分 topic/service 名称可能由 `launch` 或 ROS 参数重映射或由其他模块生成，建议在集成时：
  1. 检查 `launch/decision.launch` 与其他 launch 文件中的 remap/param；
  2. 在运行时使用 `rostopic list` / `rosservice list` / `rosnode info` 验证实际接口。

## 后续可选动作
- 生成机器可读的 JSON/YAML 接口描述，便于自动化检查或代码生成。
- 扫描并合并 `launch/decision.launch` 中的 remap 与参数到本文件。

---

> 如果需要，我可以把本文件转换为 `decision/API_REFERENCE.yaml`（或 JSON）以便程序化使用，或继续扫描 `launch` 文件合并 remap 信息。
