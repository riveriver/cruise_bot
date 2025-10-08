# IaqActionServer 使用说明

## 概述

`IaqActionServer` 是一个实现了 `task_executer/air_quality` action server 的节点，用于接收空气质量检测任务并执行数据采集。

## 功能特性

- **Action Server**: 实现 `cruise_msgs::TaskExecuterAction` 接口
- **数据采集**: 订阅 `/sensors/air_data` 话题，1秒采集1次数据
- **持续采集**: 采集60秒的空气质量数据
- **CSV存储**: 将采集的数据保存为CSV格式文件
- **进度反馈**: 实时反馈采集进度(0-100%)

## Action 接口

### 目标 (Goal)
```
string type      # 必须为 "air_quality"
string cmd       # 必须为 "start"
string id        # 任务ID，用于文件命名
string pose      # 位置信息（格式："x,y,yaw"）
string time      # 时间信息
```

### 结果 (Result)
```
string code      # "0" 表示成功，"1" 表示失败
string message   # 结果描述信息
```

### 反馈 (Feedback)
```
float32 progress # 采集进度 (0.0-100.0)
```

## 数据格式

### 订阅话题
- **话题名**: `/sensors/air_data`
- **消息类型**: `cruise_msgs::AirSensors`

### CSV文件格式
保存路径：`$(rospack find cruise_resources)/data/air_quality_{task_id}_{timestamp}.csv`

CSV列：
```
timestamp,temperature,humidity,airflow,co2,ozone_concentration,tvoc,pm1_0,pm2_5,pm10,co,no2,formaldehyde,rn,light1,light2,noise
```

## 使用方法

### 1. 启动服务
```bash
# 启动数据发布节点和Action Server
roslaunch air_sensors iaq_action_server.launch

# 或者分别启动
rosrun air_sensors ReadPubIaqData
rosrun air_sensors IaqActionServer
```

### 2. 发送任务（C++）
```cpp
#include <actionlib/client/simple_action_client.h>
#include <cruise_msgs/TaskExecuterAction.h>

// 创建客户端
actionlib::SimpleActionClient<cruise_msgs::TaskExecuterAction> client("task_executer/air_quality");
client.waitForServer();

// 创建目标
cruise_msgs::TaskExecuterGoal goal;
goal.type = "air_quality";
goal.cmd = "start";
goal.id = "task_001";
goal.pose = "1.0,2.0,0.0";
goal.time = "00:00:00";

// 发送目标
client.sendGoal(goal);
client.waitForResult();

// 获取结果
auto result = client.getResult();
ROS_INFO("Result: %s - %s", result->code.c_str(), result->message.c_str());
```

### 3. 发送任务（Python）
```python
import actionlib
from cruise_msgs.msg import TaskExecuterAction, TaskExecuterGoal

# 创建客户端
client = actionlib.SimpleActionClient('task_executer/air_quality', TaskExecuterAction)
client.wait_for_server()

# 创建目标
goal = TaskExecuterGoal()
goal.type = "air_quality"
goal.cmd = "start"
goal.id = "task_001"
goal.pose = "1.0,2.0,0.0"
goal.time = "00:00:00"

# 发送目标并等待结果
client.send_goal(goal)
result = client.wait_for_result()
```

### 4. 测试脚本
```bash
# 运行测试脚本
python3 test_iaq_action_server.py
```

## 工作流程

1. **接收任务**: Action Server 接收到 goal
2. **验证任务**: 检查 type 是否为 "air_quality"，cmd 是否为 "start"
3. **创建文件**: 在 cruise_resources/data/ 目录下创建CSV文件
4. **数据采集**: 循环60次，每次间隔1秒采集数据
5. **进度反馈**: 每次采集后发布进度反馈
6. **完成任务**: 关闭文件，返回成功结果

## 文件命名规则

CSV文件命名格式：`air_quality_{task_id}_{timestamp}.csv`

示例：`air_quality_task_001_20231008_143025.csv`

## 错误处理

- **不支持的任务类型**: 返回错误码 "1"
- **文件创建失败**: 返回错误码 "1" 
- **任务被取消**: 设置为 preempted 状态

## 依赖

- `cruise_msgs` - 消息定义
- `actionlib` - Action 框架
- `roscpp` - ROS C++ 客户端

## 注意事项

1. 确保 `/sensors/air_data` 话题有数据发布
2. 确保 `cruise_resources/data/` 目录存在且可写
3. 采集过程中可以通过 Action 机制取消任务
