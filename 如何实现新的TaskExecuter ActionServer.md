# 如何实现新的TaskExecuter ActionServer

## 概述

本文档详细介绍如何基于现有的 `IaqActionServer` 实现新的 TaskExecuter ActionServer。TaskExecuter ActionServer 是机器人任务执行系统的核心组件，负责处理特定类型的任务执行请求。

## 系统架构

### ActionServer 在系统中的角色

```mermaid
graph LR
    A[DeviceManager] --> B[TaskExecuter Client]
    B --> C[ActionServer]
    C --> D[实际任务执行]
    D --> C
    C --> B
    B --> A
```

### 消息流程

1. **Goal请求**: DeviceManager 通过 ActionLib 发送任务Goal
2. **任务执行**: ActionServer 接收Goal并执行具体任务
3. **进度反馈**: ActionServer 周期性发送执行进度
4. **结果返回**: 任务完成后返回执行结果

## IaqActionServer 分析

### 核心结构

让我们先分析现有的 `IaqActionServer` 实现：

```cpp
class IaqActionServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<cruise_msgs::TaskExecuterAction> as_;
    
    // 数据成员
    cruise_msgs::TaskExecuterFeedback feedback_;
    cruise_msgs::TaskExecuterResult result_;
    cruise_msgs::AirSensors latest_air_data_;
    
    // 任务控制
    bool is_executing_;
    ros::Time start_time_;
    std::vector<cruise_msgs::AirSensors> collected_data_;
    
    // ROS组件
    ros::Subscriber air_data_sub_;
    ros::Timer execution_timer_;

public:
    IaqActionServer(const std::string& name);
    void executeCallback(const cruise_msgs::TaskExecuterGoalConstPtr& goal);
    void airDataCallback(const cruise_msgs::AirSensors::ConstPtr& msg);
    void executionTimerCallback(const ros::TimerEvent& event);
    void saveDataToCSV(const std::vector<cruise_msgs::AirSensors>& data, 
                       const std::string& task_id);
};
```

### 关键实现细节

#### 1. 构造函数设计

```cpp
IaqActionServer::IaqActionServer(const std::string& name)
    : nh_("~"),
      as_(nh_, name, boost::bind(&IaqActionServer::executeCallback, this, _1), false),
      is_executing_(false)
{
    // 订阅传感器数据
    air_data_sub_ = nh_.subscribe("/sensors/air_data", 10, 
                                  &IaqActionServer::airDataCallback, this);
    
    // 创建定时器（但不启动）
    execution_timer_ = nh_.createTimer(ros::Duration(1.0), 
                                       &IaqActionServer::executionTimerCallback, 
                                       this, false, false);
    
    // 启动ActionServer
    as_.start();
    ROS_INFO("IaqActionServer started and ready to receive goals");
}
```

#### 2. 任务执行回调

```cpp
void IaqActionServer::executeCallback(const cruise_msgs::TaskExecuterGoalConstPtr& goal)
{
    ROS_INFO("Received air quality collection goal for task: %s", goal->id.c_str());
    
    // 验证Goal参数
    if (goal->type != "air_quality" || goal->cmd != "start") {
        result_.success = false;
        result_.message = "Invalid goal type or command";
        as_.setAborted(result_);
        return;
    }
    
    // 初始化执行状态
    is_executing_ = true;
    start_time_ = ros::Time::now();
    collected_data_.clear();
    
    // 启动定时器
    execution_timer_.start();
    
    // 执行60秒的数据收集
    executeAirQualityCollection(goal);
}
```

#### 3. 数据收集和处理

```cpp
void IaqActionServer::executeAirQualityCollection(const cruise_msgs::TaskExecuterGoalConstPtr& goal)
{
    ros::Duration collection_duration(60.0);  // 60秒收集时间
    ros::Rate rate(1.0);  // 1Hz 采样率
    
    while (is_executing_ && (ros::Time::now() - start_time_) < collection_duration) {
        // 更新进度反馈
        feedback_.progress = static_cast<int>(
            ((ros::Time::now() - start_time_).toSec() / 60.0) * 100);
        feedback_.status = "Collecting air quality data...";
        as_.publishFeedback(feedback_);
        
        // 检查是否被取消
        if (as_.isPreemptRequested() || !ros::ok()) {
            as_.setPreempted();
            is_executing_ = false;
            execution_timer_.stop();
            return;
        }
        
        rate.sleep();
    }
    
    // 任务完成
    is_executing_ = false;
    execution_timer_.stop();
    
    // 保存数据并返回结果
    saveDataToCSV(collected_data_, goal->id);
    result_.success = true;
    result_.message = "Air quality data collection completed successfully";
    as_.setSucceeded(result_);
}
```

## 实现新的ActionServer步骤

### 步骤1: 定义ActionServer类

```cpp
// 示例：相机拍照ActionServer
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cruise_msgs/TaskExecuterAction.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraActionServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<cruise_msgs::TaskExecuterAction> as_;
    
    // 反馈和结果
    cruise_msgs::TaskExecuterFeedback feedback_;
    cruise_msgs::TaskExecuterResult result_;
    
    // 任务状态
    bool is_executing_;
    std::string task_id_;
    
    // ROS组件
    ros::Subscriber image_sub_;
    
    // 相机相关
    sensor_msgs::ImageConstPtr latest_image_;
    std::vector<cv::Mat> captured_images_;

public:
    CameraActionServer(const std::string& name);
    ~CameraActionServer() = default;
    
    // 回调函数
    void executeCallback(const cruise_msgs::TaskExecuterGoalConstPtr& goal);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    
    // 任务执行
    void executeCameraTask(const cruise_msgs::TaskExecuterGoalConstPtr& goal);
    void captureImages(int num_images, double interval);
    void saveImages(const std::string& task_id);
};
```

### 步骤2: 实现构造函数

```cpp
CameraActionServer::CameraActionServer(const std::string& name)
    : nh_("~"),
      as_(nh_, name, boost::bind(&CameraActionServer::executeCallback, this, _1), false),
      is_executing_(false)
{
    // 订阅相机图像话题
    image_sub_ = nh_.subscribe("/camera/image_raw", 1, 
                               &CameraActionServer::imageCallback, this);
    
    // 启动ActionServer
    as_.start();
    ROS_INFO("CameraActionServer started and ready to receive goals");
}
```

### 步骤3: 实现任务执行逻辑

```cpp
void CameraActionServer::executeCallback(const cruise_msgs::TaskExecuterGoalConstPtr& goal)
{
    ROS_INFO("Received camera task goal for task: %s", goal->id.c_str());
    
    // 验证Goal参数
    if (goal->type != "camera_capture" || goal->cmd != "start") {
        result_.success = false;
        result_.message = "Invalid goal type or command for camera task";
        as_.setAborted(result_);
        return;
    }
    
    // 初始化执行状态
    is_executing_ = true;
    task_id_ = goal->id;
    captured_images_.clear();
    
    // 执行相机任务
    executeCameraTask(goal);
}

void CameraActionServer::executeCameraTask(const cruise_msgs::TaskExecuterGoalConstPtr& goal)
{
    try {
        // 解析任务参数（从goal->pose或其他参数中获取）
        int num_images = 5;        // 拍摄5张照片
        double interval = 2.0;     // 每2秒拍摄一张
        
        ROS_INFO("Starting camera capture: %d images with %.1f second interval", 
                 num_images, interval);
        
        // 执行拍照任务
        captureImages(num_images, interval);
        
        // 保存图像
        saveImages(task_id_);
        
        // 任务成功完成
        result_.success = true;
        result_.message = "Camera capture task completed successfully";
        as_.setSucceeded(result_);
        
    } catch (const std::exception& e) {
        result_.success = false;
        result_.message = std::string("Camera task failed: ") + e.what();
        as_.setAborted(result_);
    }
    
    is_executing_ = false;
}
```

### 步骤4: 实现具体功能

```cpp
void CameraActionServer::captureImages(int num_images, double interval)
{
    ros::Rate rate(1.0 / interval);
    
    for (int i = 0; i < num_images && is_executing_; ++i) {
        // 检查是否被取消
        if (as_.isPreemptRequested() || !ros::ok()) {
            as_.setPreempted();
            is_executing_ = false;
            return;
        }
        
        // 等待新图像
        if (latest_image_) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_image_, 
                                                                   sensor_msgs::image_encodings::BGR8);
                captured_images_.push_back(cv_ptr->image.clone());
                
                // 更新进度反馈
                feedback_.progress = static_cast<int>(((i + 1.0) / num_images) * 100);
                feedback_.status = "Captured image " + std::to_string(i + 1) + "/" + std::to_string(num_images);
                as_.publishFeedback(feedback_);
                
                ROS_INFO("Captured image %d/%d", i + 1, num_images);
                
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                throw std::runtime_error("Failed to convert image");
            }
        } else {
            ROS_WARN("No image available, waiting...");
            // 等待图像数据
            ros::Duration(0.5).sleep();
            --i;  // 重试当前图像
            continue;
        }
        
        if (i < num_images - 1) {  // 最后一张不需要等待
            rate.sleep();
        }
    }
}

void CameraActionServer::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    latest_image_ = msg;
}

void CameraActionServer::saveImages(const std::string& task_id)
{
    std::string save_dir = "/tmp/camera_tasks/" + task_id + "/";
    
    // 创建目录
    std::string mkdir_cmd = "mkdir -p " + save_dir;
    system(mkdir_cmd.c_str());
    
    // 保存图像
    for (size_t i = 0; i < captured_images_.size(); ++i) {
        std::string filename = save_dir + "image_" + std::to_string(i + 1) + ".jpg";
        cv::imwrite(filename, captured_images_[i]);
        ROS_INFO("Saved image: %s", filename.c_str());
    }
    
    ROS_INFO("All images saved to: %s", save_dir.c_str());
}
```

### 步骤5: 创建主函数

```cpp
// main函数 (camera_action_server_main.cpp)
#include "CameraActionServer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_action_server");
    
    CameraActionServer server("task_executer/camera_capture");
    
    ROS_INFO("Camera Action Server is running...");
    ros::spin();
    
    return 0;
}
```

### 步骤6: 配置CMakeLists.txt

```cmake
# 在 CMakeLists.txt 中添加
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  cruise_msgs
  sensor_msgs
  cv_bridge
  # 其他依赖...
)

find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(camera_action_server
  src/CameraActionServer.cpp
  src/camera_action_server_main.cpp
)

target_link_libraries(camera_action_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(camera_action_server 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
```

## 集成到DeviceManager

### 1. 添加ActionClient

在 `DeviceManager.hpp` 中添加：

```cpp
private:
    std::unique_ptr<TaskExecuter> camera_capture_;  // 新增相机ActionClient
```

### 2. 初始化Client

在 `DeviceManagerInit.cpp` 构造函数中：

```cpp
DeviceManager::DeviceManager(ros::NodeHandle &nh)
    : // 其他初始化...
      camera_capture_(new TaskExecuter("task_executer/camera_capture"))  // 新增
{
    // 其他初始化代码...
}
```

### 3. 添加回调函数

在 `DeviceManager.hpp` 中声明：

```cpp
void CameraCaptureTaskDone(const actionlib::SimpleClientGoalState &state,
                           const cruise_msgs::TaskExecuterResultConstPtr &result);
```

在 `DeviceManager.cpp` 中实现：

```cpp
void DeviceManager::CameraCaptureTaskDone(const actionlib::SimpleClientGoalState &state,
                                          const cruise_msgs::TaskExecuterResultConstPtr &result)
{
    ROS_INFO("CameraCaptureTask got state [%s]", state.toString().c_str());
    if (!result)
        return;
    ROS_INFO("CameraCaptureTask got result: %s", result->message.c_str());
    
    // 重置标志位（如果使用）
    camera_capture_goal_sent_ = false;
    
    // 根据任务类型决定下一步状态
    ROS_INFO("Camera capture completed, transitioning to publish reverse route");
    task_state_ = TaskStatus::PubReverseRoute;
}
```

### 4. 扩展ExecuteActionBehavior

在 `ExecuteActionBehavior()` 函数中添加新的任务类型：

```cpp
bool DeviceManager::ExecuteActionBehavior()
{
    TaskInfo task = cur_task_;
    
    /* Camera Capture Tasks */
    if (task.actions[task.action_index] == "Photo_Documentation" ||
        task.actions[task.action_index] == "Visual_Inspection" ||
        task.actions[task.action_index] == "Progress_Recording")
    {
        if (!camera_capture_goal_sent_) {
            ROS_INFO("Sending camera capture Goal once, then waiting for result");
            auto pose = device_pose_;
            cruise_msgs::TaskExecuterGoal req;
            req.type = "camera_capture";
            req.cmd = "start";
            req.id = cur_task_.id;
            std::stringstream ss;
            ss << pose.x << "," << pose.y << "," << pose.yaw;
            req.pose = ss.str();
            req.time = "00:00:00";
            camera_capture_->sendGoal(req,
                                      boost::bind(&DeviceManager::CameraCaptureTaskDone, this, _1, _2));
            camera_capture_goal_sent_ = true;
            ROS_INFO("Camera capture Goal sent, waiting for completion");
        } else {
            ROS_DEBUG("Camera capture Goal already sent, waiting for completion");
        }
        return true;
    }
    
    /* 其他现有任务类型... */
    
    return false;
}
```

## 通用ActionServer模板

为了简化新ActionServer的开发，这里提供一个通用模板：

```cpp
#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cruise_msgs/TaskExecuterAction.h>

template<typename TaskData>
class GenericTaskActionServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<cruise_msgs::TaskExecuterAction> as_;
    
    cruise_msgs::TaskExecuterFeedback feedback_;
    cruise_msgs::TaskExecuterResult result_;
    
    bool is_executing_;
    std::string expected_task_type_;
    
protected:
    // 子类需要实现的纯虚函数
    virtual bool validateGoal(const cruise_msgs::TaskExecuterGoalConstPtr& goal) = 0;
    virtual void executeTask(const cruise_msgs::TaskExecuterGoalConstPtr& goal) = 0;
    virtual void cleanupTask() = 0;

public:
    GenericTaskActionServer(const std::string& name, const std::string& task_type)
        : nh_("~"),
          as_(nh_, name, boost::bind(&GenericTaskActionServer::executeCallback, this, _1), false),
          is_executing_(false),
          expected_task_type_(task_type)
    {
        as_.start();
        ROS_INFO("%s ActionServer started for task type: %s", name.c_str(), task_type.c_str());
    }
    
    virtual ~GenericTaskActionServer() = default;

private:
    void executeCallback(const cruise_msgs::TaskExecuterGoalConstPtr& goal)
    {
        if (!validateGoal(goal)) {
            result_.success = false;
            result_.message = "Invalid goal for " + expected_task_type_;
            as_.setAborted(result_);
            return;
        }
        
        is_executing_ = true;
        
        try {
            executeTask(goal);
        } catch (const std::exception& e) {
            result_.success = false;
            result_.message = std::string("Task execution failed: ") + e.what();
            as_.setAborted(result_);
        }
        
        cleanupTask();
        is_executing_ = false;
    }

protected:
    // 工具函数供子类使用
    void publishProgress(int progress, const std::string& status) {
        feedback_.progress = progress;
        feedback_.status = status;
        as_.publishFeedback(feedback_);
    }
    
    void setTaskSucceeded(const std::string& message) {
        result_.success = true;
        result_.message = message;
        as_.setSucceeded(result_);
    }
    
    void setTaskFailed(const std::string& message) {
        result_.success = false;
        result_.message = message;
        as_.setAborted(result_);
    }
    
    bool isPreemptRequested() const {
        return as_.isPreemptRequested() || !ros::ok();
    }
    
    void setPreempted() {
        as_.setPreempted();
    }
};
```

## 最佳实践

### 1. 错误处理
- 使用try-catch包装可能失败的操作
- 提供详细的错误信息
- 正确设置ActionResult状态

### 2. 进度反馈
- 定期发布进度更新
- 提供有意义的状态描述
- 使用百分比表示完成度

### 3. 取消处理
- 定期检查是否被取消
- 快速响应取消请求
- 清理资源

### 4. 参数配置
- 通过ROS参数服务器配置任务参数
- 支持运行时参数调整
- 提供合理的默认值

### 5. 日志记录
- 记录关键执行步骤
- 区分不同日志级别
- 包含任务ID等关键信息

## 调试和测试

### 1. 命令行测试

```bash
# 启动ActionServer
rosrun your_package camera_action_server

# 使用命令行工具测试
rostopic pub /task_executer/camera_capture/goal cruise_msgs/TaskExecuterActionGoal "
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
goal_id:
  stamp: {secs: 0, nsecs: 0}
  id: ''
goal:
  type: 'camera_capture'
  cmd: 'start'
  id: 'test_task_001'
  pose: '0,0,0'
  time: '00:00:00'
"
```

### 2. 监控状态

```bash
# 监控反馈
rostopic echo /task_executer/camera_capture/feedback

# 监控结果
rostopic echo /task_executer/camera_capture/result
```

## 总结

实现新的TaskExecuter ActionServer的关键要点：

1. **遵循ActionLib协议**: 正确实现Goal、Feedback、Result机制
2. **状态管理**: 维护清晰的执行状态和生命周期
3. **错误处理**: 完善的异常处理和错误报告
4. **集成设计**: 与DeviceManager状态机良好集成
5. **可扩展性**: 使用模板和继承提高代码复用

通过参考IaqActionServer的设计模式，可以快速实现满足系统要求的新ActionServer，确保与现有架构的兼容性和一致性。
