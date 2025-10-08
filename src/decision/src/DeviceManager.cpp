#include <iomanip>
#include <sstream>
#include <chrono>
#include <fstream>
#include <tf/transform_datatypes.h>
#include "DeviceManagerMqtt.hpp"
#include "DeviceManagerMotion.hpp"
#include "DeviceManagerTask.hpp"
#include "DeviceManager.hpp"

uint8_t *DeviceManager::cost_translation_ = nullptr;

// Constructor, destructor and init moved to DeviceManagerInit.cpp

void DeviceManager::run()
{
  using goalState = actionlib::SimpleClientGoalState;
  ros::Rate r(ros::Duration(0.1));
  while (ros::ok())
  {
    ros::spinOnce();
  UpdateSystemMode(system_mode_);
    r.sleep();
  }
}

void DeviceManager::ExecuteMoveBehavior()
{
  if (motion_)
    motion_->ExecuteMoveBehavior();
}

void DeviceManager::BuildingServiceDone(const actionlib::SimpleClientGoalState &state,
                                       const cruise_msgs::TaskExecuterResultConstPtr &result)
{
  move_permit_ = true;
  ROS_INFO("BuildingService got state [%s]", state.toString().c_str());
  if (!result)
    return;
  ROS_INFO("BuildingService got result");
  //  send_exe(current_index_);
}

void DeviceManager::AirQualityDone(const actionlib::SimpleClientGoalState &state,
                                  const cruise_msgs::TaskExecuterResultConstPtr &result)
{
  ROS_INFO("AirQuality got state [%s]", state.toString().c_str());
  if (!result)
    return;
  ROS_INFO("AirQuality got result");
  
  // 重置air_quality Goal发送标志位
  air_quality_goal_sent_ = false;
  
  // 空气质量检测完成后，转换状态到发布反向路线
  ROS_INFO("Air quality detection completed, transitioning to publish reverse route");
  task_state_ = TaskStatus::PubReverseRoute;
  //  send_exe(current_index_);
}

void DeviceManager::AppControlBehavior()
{
  AppMoveCmd move = cmd_move;
  float max_vel = move.speed * 1.0;
  geometry_msgs::Twist vel;

  // stop
  if (move.stop || (std::time(0) - move.rev_time > 2))
  {
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;
    cmd_vel_pub_.publish(vel);
    return;
  }

  // 旋转
  if (!move.up && !move.down)
  {

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;

    if (move.left)
      vel.angular.z += 0.5;
    if (move.right)
      vel.angular.z -= 0.5;
    if (!move.left && !move.right)
      vel.angular.z = 0.0;
    vel.angular.z = vel.angular.z < -max_vel ? -max_vel : vel.angular.z;
    vel.angular.z = vel.angular.z > max_vel ? max_vel : vel.angular.z;
    cmd_vel_pub_.publish(vel);
  }
  else
  {
    vel.angular.z = 0.0;
    // 直行
    if (move.up)
      vel.linear.x += 0.5;
    if (move.down)
      vel.linear.x -= 0.5;
    if (!move.up && !move.down)
      vel.linear.x = 0.0;
    vel.linear.x = vel.linear.x < -max_vel ? -max_vel : vel.linear.x;
    vel.linear.x = vel.linear.x > max_vel ? max_vel : vel.linear.x;
    // 转向
    if (move.left)
      vel.linear.y += 0.5;
    if (move.right)
      vel.linear.y -= 0.5;
    if (!move.left && !move.right)
      vel.linear.y = 0.0;
    vel.linear.y = vel.linear.y < -max_vel ? -max_vel : vel.linear.y;
    vel.linear.y = vel.linear.y > max_vel ? max_vel : vel.linear.y;
    cmd_vel_pub_.publish(vel);
  }
}

void DeviceManager::RecordRouteBehavior()
{
  auto pose = robot_pose();
  if (!pose)
    return;

  RecordPathWaypoint();
}

void DeviceManager::AutoNavBehavior()
{
  /* task action client */
  switch (task_state_)
  {
  case TaskStatus::PubTask:
    PubTask();
    break;
  case TaskStatus::PubRoute:
    PubRoute();
    break;
  case TaskStatus::GoForwardRoute:
    ExecuteMoveBehavior();
    break;
  case TaskStatus::ExecuteAction:
    ExecuteActionBehavior();
    break;
  case TaskStatus::PubReverseRoute:
    PubReverseRoute();
    break;
  case TaskStatus::GoReverseRoute:
    ExecuteMoveBehavior();
    break;
  case TaskStatus::CompletedRoute:
    // 任务完成，准备进入下一个任务
    HandleRouteCompleted();
    break;
  default:
    ROS_ERROR("unkown task step:%d", static_cast<int>(task_state_));
    break;
  }
}

void DeviceManager::UpdateSystemMode(SystemMode mode)
{
  CheckReadyTask();

  switch (mode)
  {
  case SystemMode::Idle:
    // do nothing, cancel goal and reset value in event
    break;
  case SystemMode::AppControl:
    AppControlBehavior();
    break;
  case SystemMode::RecordRoute:
    RecordRouteBehavior();
    break;
  case SystemMode::AutoNav:
    AutoNavBehavior();
    break;
  case SystemMode::BackHome:
    // TODO
    break;
  default:
    ROS_ERROR("UpdateSystemMode:unkown mode");
    break;
  }
}

bool DeviceManager::ExecuteActionBehavior()
{

  TaskInfo task = cur_task_;

  // /* BuildingService */
  // if (
  //     task.actions[task.action_index] == "HVAC_Duct" ||
  //     task.actions[task.action_index] == "EL_Trunking" ||
  //     task.actions[task.action_index] == "FS_Pipe" ||
  //     task.actions[task.action_index] == "DR_WP_Pipe" ||
  //     task.actions[task.action_index] == "EL_Lighting")
  // {
  //   auto pose = device_pose_;
  //   if (bs_pose_.x = 0 && bs_pose_.y == 0 && bs_pose_.yaw == 0)
  //   {
  //     bs_pose_ = pose;
  //     cruise_msgs::TaskExecuterGoal req;
  //     req.type = "building_service";
  //     req.cmd = "start";
  //     req.id = cur_task_.id;
  //     std::stringstream ss;
  //     ss << pose.x << "," << pose.y << "," << pose.yaw;
  //     req.pose = ss.str();
  //     req.time = "00:00:00";
  //     building_service_->sendGoal(req,
  //                                 boost::bind(&DeviceManager::BuildingServiceDone, this, _1, _2));
  //     return true;
  //   }
  //   auto len = std::hypot(bs_pose_.x - pose.x, bs_pose_.y - pose.y);
  //   auto agu = std::abs(bs_pose_.yaw - pose.yaw);
  //   if (len < 1.0 && agu < 10 * DEG2RAD)
  //     return true;

  //   // 确定进入任务执行阶段
  //   ROS_INFO("task:%s, enter task execution phase.", cur_task_.id.c_str());
  //   // 调用goto_ctrl_使机器人暂停移动
  //   move_permit_ = false;
  //   goto_ctrl_->cancelGoal();
  //   pub_zero_vel();

  //   // 触发拍照服务（异步调用）
  //   cruise_msgs::TaskExecuterGoal req;
  //   req.type = "building_service";
  //   req.cmd = "trigger";
  //   req.id = cur_task_.id;
  //   std::stringstream ss;
  //   ss << pose.x << "," << pose.y << "," << pose.yaw;
  //   req.pose = ss.str();
  //   req.time = "00:00:00";
  //   building_service_->sendGoal(req,
  //                               boost::bind(&DeviceManager::BuildingServiceDone, this, _1, _2));

  //   bs_pose_ = pose;
  //   return true;
  // }

  /* AirQuality */
  if (
      task.actions[task.action_index] == "IAQ_Inspection" ||
      task.actions[task.action_index] == "Lux_Level" ||
      task.actions[task.action_index] == "Noise_Level" ||
      task.actions[task.action_index] == "Temperature" ||
      task.actions[task.action_index] == "Humidity" ||
      task.actions[task.action_index] == "Air_Flow")
  {
    // 只发送一次air_quality Goal，然后等待结果
    if (!air_quality_goal_sent_) {
      ROS_INFO("Sending air_quality Goal once, then waiting for result");
      auto pose = device_pose_;
      cruise_msgs::TaskExecuterGoal req;
      req.type = "air_quality";
      req.cmd = "start";
      req.id = cur_task_.id;
      std::stringstream ss;
      ss << pose.x << "," << pose.y << "," << pose.yaw;
      req.pose = ss.str();
      req.time = "00:00:00";
      air_quality_->sendGoal(req,
                             boost::bind(&DeviceManager::AirQualityDone, this, _1, _2));
      air_quality_goal_sent_ = true;  // 设置标志位，避免重复发送
      ROS_INFO("Air quality Goal sent, waiting for completion");
    } else {
      ROS_DEBUG("Air quality Goal already sent, waiting for completion");
    }
    return true;
  }
  return false;
}

std::string DeviceManager::TimeToYMDHMS(const std::time_t &time)
{
  std::tm *ltm = std::localtime(&time);
  std::stringstream ss;
  ss << std::put_time(ltm, "%Y-%m-%d %H:%M:%S");
  return ss.str();
}

bool DeviceManager::PubTask()
{
  if (task_helper_)
    return task_helper_->PubTask();
  return false;
}

bool DeviceManager::PubRoute()
{
  if (task_helper_)
    return task_helper_->PubRoute();
  return false;
}

uint8_t DeviceManager::AddTasKInQueue(const TaskInfo &obj)
{
  if (task_helper_)
    return task_helper_->AddTasKInQueue(obj);
  return 1;
}

uint8_t DeviceManager::MoveTaskFrontQueue(const TaskInfo &obj)
{
  if (task_helper_)
    return task_helper_->MoveTaskFrontQueue(obj);
  return 1;
}

uint8_t DeviceManager::RemoveTaskInQueue(const TaskInfo &obj)
{
  if (task_helper_)
    return task_helper_->RemoveTaskInQueue(obj);
  return 1;
}

TaskInfo DeviceManager::PeekFrontReadyTasks()
{
  if (task_helper_)
    return task_helper_->PeekFrontReadyTasks();
  return {};
}

TaskInfo DeviceManager::PopFrontReadyTasks()
{
  if (task_helper_)
    return task_helper_->PopFrontReadyTasks();
  return {};
}

std::string DeviceManager::GetTaskInfoSimple()
{
  if (task_helper_)
    return task_helper_->GetTaskInfoSimple();
  return std::string();
}

std::string DeviceManager::GetTaskInfo()
{
  if (task_helper_)
    return task_helper_->GetTaskInfo();
  return std::string();
}

void DeviceManager::PackDeviceInfo(Json::Value &result)
{
  int task_time = (cur_task_.start_time == 0) ? 0 : (std::time(0) - cur_task_.start_time);
  result["RunningTime"] = std::to_string(task_time);
  result["currentTaskId"] = cur_task_.id;
  result["currentTaskStatus"] = cur_task_.state;
  result["taskProcess"] = cur_task_.progress;
  result["isControl"] = (system_mode_ == SystemMode::AppControl) ? 1 : 0;
  result["isliveStream"] = is_live_stream;
  result["isDisplayMode"] = is_iaq_upload;
  result["isCharge"] = charge_state_;
  result["battery"] = battery_level_;
  result["error"] = 0;
  result["floor"] = 2;
  // TODO
  // result["RobotMode"] = std::to_string(base_mode);
}

void DeviceManager::PubAckMsg(MqttMsgHead &msg_head, Json::Value payload)
{
  if (!mqtt_client_)
  {
    ROS_ERROR("PubAckMsg: mqtt_client_ not initialized");
    return;
  }

  Json::Value data;
  data["code"] = msg_head.code;
  data["message"] = msg_head.msg;
  data["payload"] = payload;

  Json::Value root;
  root["device_id"] = "prototypes001";
  root["timestamp"] = std::to_string(std::time(0));
  root["type"] = msg_head.type;
  root["messageId"] = msg_head.id;
  root["data"] = data;

  Json::StreamWriterBuilder writer;
  std::string json_string = Json::writeString(writer, root);
  // Defensive checks and logging to help diagnose MOSQ_ERR_INVAL (rc=3)
  if (msg_head.topic.empty())
  {
    ROS_ERROR("PubAckMsg: empty topic for messageId=%s", msg_head.id.c_str());
    return;
  }
  if (msg_head.topic.size() > 65535)
  {
    ROS_ERROR("PubAckMsg: topic too long (%zu)", msg_head.topic.size());
    return;
  }
  ROS_DEBUG("PubAckMsg: publishing to topic='%s' qos=%d payload_len=%zu", msg_head.topic.c_str(), msg_head.qos, json_string.size());
  bool ok = mqtt_client_->publish(msg_head.topic, json_string, msg_head.qos);
  if (!ok)
    ROS_ERROR("Failed to publish MQTT message to %s", msg_head.topic.c_str());
  else
    ROS_DEBUG("[pub] %s: %s", msg_head.topic.c_str(), json_string.c_str());
}

void DeviceManager::PackDevicePose(Json::Value &result)
{
  std::stringstream pose_stream;
  pose_stream << device_pose_.x << "," << device_pose_.y << "," << device_pose_.yaw;
  result["location"] = pose_stream.str();
}

void DeviceManager::UploadSlowInfoCallback(const ros::TimerEvent &)
{
  ROS_INFO("%s", GetTaskInfo().c_str());
}

void DeviceManager::UploadFastInfoCallback(const ros::TimerEvent &)
{
  MqttMsgHead msg_head;
  msg_head.code = 0;
  msg_head.id = "0";
  msg_head.topic = "response/prototypes001/move";
  msg_head.type = "move";
  msg_head.qos = 0;
  Json::Value payload;
  PackDevicePose(payload);
  PubAckMsg(msg_head, payload);
}

void DeviceManager::PackAirData(const cruise_msgs::AirSensors &data, Json::Value &result)
{
  result["taskId"] = (cur_task_.id != "0") ? cur_task_.id : "default";
  result["floor"] = (cur_task_.id != "0") ? cur_task_.areas[cur_task_.pose_index].floor : 0;
  result["area"] = (cur_task_.id != "0") ? cur_task_.areas[cur_task_.pose_index].num : 0;
  result["spot"] = 0;

  result["temperature"] = data.temperature;
  result["humidity"] = data.humidity;
  result["airflow"] = data.airflow;
  result["co2"] = data.co2;
  result["o3"] = data.ozone_concentration;
  result["tvoc"] = data.tvoc;
  result["pm1_0"] = data.pm1_0;
  result["pm2_5"] = data.pm2_5;
  result["pm10"] = data.pm10;
  result["co"] = data.co;
  result["no2"] = data.no2;
  result["hcho"] = data.formaldehyde;
  result["rn"] = data.rn;
  result["lux"] = data.light1;
  result["lux2"] = data.light2;
  result["noise"] = data.noise;
}

// UploadAirData and UploadAirCallback implemented in DeviceManagerMqtt.cpp


void DeviceManager::SubBatteryCallback(const ranger_msgs::BatteryState::ConstPtr &msg)
{
  battery_level_ = msg->percentage;
  charge_state_ = (msg->current > 15);
}

void DeviceManager::SubDevicePoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  tf::Quaternion q;
  double roll, pitch, yaw;
  nav_msgs::Odometry odom = *msg;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double yaw_degrees = yaw * (180.0 / M_PI);
  device_pose_.x = odom.pose.pose.position.x;
  device_pose_.y = odom.pose.pose.position.y;
  device_pose_.yaw = yaw_degrees;
}

void DeviceManager::SubAirDataCallback(const cruise_msgs::AirSensors::ConstPtr &msg)
{
  iaq_data = *msg;
}

void DeviceManager::SubAirResultCallback(const cruise_msgs::AirSensors::ConstPtr &msg)
{
  iaq_data = *msg;
  MqttMsgHead msg_head;
  msg_head.code = 0;
  msg_head.id = "0202";
  msg_head.topic = "response/prototypes001/sensor";
  msg_head.type = "sensor_result";
  Json::Value payload;
  PackAirData(iaq_data, payload);
  PubAckMsg(msg_head, payload);
}

/* MQTT */
void DeviceManager::SetAirUpload(bool enable)
{
  if (enable)
  {
    upload_sensor_timer_.start();
    is_iaq_upload = true;
    UploadAirData();
    // sound_play.playWave(resource_path_ + "/sounds/A0.wav");
    ROS_INFO("[DECISION]upload air:enable");
  }
  else
  {
    upload_sensor_timer_.stop();
    is_iaq_upload = false;
    ROS_INFO("[DECISION]upload air:disable");
  }
}

void DeviceManager::SetLiveStream(bool enable)
{
  if (enable)
  {
    is_live_stream = true;
    // sound_play.playWave(resource_path_ + "/sounds/L0.wav");
    ROS_INFO("[DECISION]live_stream: enable");
  }
  else
  {
    is_live_stream = false;
    ROS_INFO("[DECISION]live_stream: disable");
  }
}

void DeviceManager::SetStopMode(bool enable)
{
  if (enable)
  {
    system_mode_ = SystemMode::Idle;
    ROS_INFO("[DECISION]stop mode: enable");
  }
  else
  {
    system_mode_ = SystemMode::AppControl;
    ROS_INFO("[DECISION]stop mode: disable");
  }
}

void DeviceManager::SetControlMode(bool enable)
{
  if (enable)
  {
    system_mode_ = SystemMode::AppControl;
    ROS_INFO("[DECISION]control mode: enable");
  }
  else
  {
    system_mode_ = SystemMode::AutoNav;
    ROS_INFO("[DECISION]control mode: disable");
  }
}

void DeviceManager::SetHomeMode(bool enable)
{
  if (enable)
  {
    system_mode_ = SystemMode::BackHome;
    ROS_INFO("[DECISION]home mode: enable");
  }
  else
  {
    system_mode_ = SystemMode::AppControl;
    ROS_INFO("[DECISION]home mode: disable");
  }
}

// mqtt2ros* callbacks implemented in DeviceManagerMqtt.cpp

// mqtt2rosTaskCallback implemented in DeviceManagerMqtt.cpp

// mqtt2rosSensorCallback implemented in DeviceManagerMqtt.cpp

void DeviceManager::CheckReadyTask()
{
  if (task_helper_) task_helper_->CheckReadyTask();
}

/**
 * @brief 发布任务
 *
 * 该函数负责从任务队列中取出当前可执行的任务，并发布该任务。
 *
 * 在任务执行前，会检查任务队列是否为空，如果不为空，则取出队列中的第一个可执行任务，
 * 并设置任务状态为发布路径。然后更新当前任务的索引和当前路径。
 *
 * 任务-->动作群-->动作-->航线群-->航线-->航点
 * 示教路径模式：
 * @param 无
 *
 * @return 无
 */
// PubTask and PubRoute moved to DeviceManagerTask.cpp

void DeviceManager::RecordPathWaypoint(){  
    // 确保了路径记录以机器人的当前姿态开始
  geometry_msgs::PoseStamped ps;
  auto pose = robot_pose();
  if (!pose)
    return;

  if (record_path_.poses.empty())
  {
    ps.pose = pose.value();
    record_path_.poses.emplace_back(ps);
    return;
  }

  auto len = std::hypot(record_path_.poses.back().pose.position.x - pose->position.x,
                        record_path_.poses.back().pose.position.y - pose->position.y);
  auto agu = std::abs(tf2::getYaw(record_path_.poses.back().pose.orientation) - tf2::getYaw(pose->orientation));
  if (len < RECORD_PATH_LEN_DENS && agu < RECORD_PATH_AGU_DENS)
    return;
  ps.pose = pose.value();
  record_path_.poses.emplace_back(ps);
  if (motion_)
    motion_->publishRecordPath(record_path_);
}

auto DeviceManager::robot_pose() -> std::optional<geometry_msgs::Pose>
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_robot_pose->lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("tf error: %s", ex.what());
    return std::nullopt;
  }

  geometry_msgs::Pose result;
  result.position.x = transformStamped.transform.translation.x;
  result.position.y = transformStamped.transform.translation.y;
  result.orientation = transformStamped.transform.rotation;
  return std::make_optional(result);
}

auto DeviceManager::TaskCmdCallback(cruise_msgs::TaskService::Request &req, cruise_msgs::TaskService::Response &resp) -> bool
{
  switch (req.type)
  {
  case cruise_msgs::TaskService::Request::EXECUTE:
  {
    if (req.command == cruise_msgs::TaskService::Request::START)
    {
      stop_move();

      // get path name
      auto dir = req.dir;
      if (dir == "")
        dir = resource_path_ + "/path/";
      file_path_ = dir + req.path_name;
      if (!check_file_exist(file_path_))
      {
        resp.message = file_path_ + " not exist, ignore start command.";
        return true;
      }
      if (!read_file(file_path_))
      {
        resp.message = "Read path in file failed.";
        return true;
      }

  system_mode_ = SystemMode::AutoNav;
  task_state_ = TaskStatus::ExecuteAction;
  if (motion_)
        motion_->StartExecuteFromFront();
      resp.message = "Start new path!";
      return true;
    }

    resp.message = "Wrong exe service command, do nothing.";
    return true;
  }
  case cruise_msgs::TaskService::Request::RECORD:
  {
    switch (req.command)
    {
    case cruise_msgs::TaskService::Request::START:
    {
      // get route name
      auto name = req.path_name;
      if (name == "")
      {
        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream ss;
        ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
        name = ss.str();
      }
      auto dir = req.dir;
      if (dir == "")
      {
        dir = resource_path_ + "/path/";
      }
      file_path_ = dir + name;
      if (check_file_exist(file_path_))
      {
        resp.message = "File already exist, ignore this operation.";
        return true;
      }
      record_path_.poses.clear();
      record_points_.clear();
      system_mode_ = SystemMode::RecordRoute;
      resp.message = "Start recording path, save to " + file_path_ + ".";
      return true;
    }
    case cruise_msgs::TaskService::Request::KEEP_TEACH:
    {
      if (system_mode_ != SystemMode::RecordRoute)
      {
        resp.message = "Not RecordRoute Mode";
        return true;
      }
      std::vector<nav_msgs::Path> paths{record_path_};
      if (!write_file(file_path_, paths))
      {
        resp.message = "Write file failed:" + file_path_;
      }
      else
      {
        resp.message = "Keep teach path, save successful.";
      }
      record_path_.poses.clear();
      record_points_.clear();
      system_mode_ = SystemMode::AutoNav;
      return true;
    }
    case cruise_msgs::TaskService::Request::DISCARD:
    {
      if (system_mode_ != SystemMode::RecordRoute)
      {
        resp.message = "Not RecordRoute Mode";
        return true;
      }
      system_mode_ = SystemMode::AutoNav;
      resp.message = "Do not keep path, discard recorded data.";
      record_path_.poses.clear();
      record_points_.clear();
      return true;
    }
    default:
      ROS_ERROR("Illegal service command in record %d", req.command);
      break;
    }
    return true;
  }
 }
 ROS_ERROR("Illegal service type: %d", req.command);
 return true;
}

auto DeviceManager::send_goto(size_t const &index) -> bool
{
  if (motion_)
    return motion_->send_goto(index);
  return false;
}

void DeviceManager::pub_zero_vel()
{
  if (motion_)
    motion_->pub_zero_vel();
}

void DeviceManager::pause_move()
{
  if (motion_)
    motion_->pause_move();
}

void DeviceManager::stop_move()
{
  if (motion_)
    motion_->stop_move();
}

void DeviceManager::resume_move()
{
  if (motion_)
    motion_->resume_move();
}

void DeviceManager::start_reverse_execution()
{
  if (motion_)
    motion_->StartExecuteReverse();
}

bool DeviceManager::PubReverseRoute()
{
  ROS_INFO("Publishing reverse route to return to start point");
  
  // 启动反向路径执行
  if (motion_) {
    if (motion_->StartExecuteReverse()) {
      task_state_ = TaskStatus::GoReverseRoute;
      ROS_INFO("Started reverse route execution");
      return true;
    } else {
      ROS_ERROR("Failed to start reverse route execution");
      return false;
    }
  }
  
  ROS_ERROR("Motion module not available");
  return false;
}

void DeviceManager::HandleRouteCompleted()
{
  ROS_INFO("Current task completed, transitioning to next task");
  
  // 从任务队列中移除已完成的任务
  if (task_helper_) {
    task_helper_->PopFrontReadyTasks();
    ROS_INFO("Completed task removed from queue");
  }
  
  // 重置air_quality Goal发送标志位，为下个任务做准备
  air_quality_goal_sent_ = false;
  
  // 重置任务状态到发布任务，准备执行下一个任务
  task_state_ = TaskStatus::PubTask;
  
  // 这里可以添加其他任务完成后的清理工作
  // 例如：重置相关变量、发送完成通知等
}

auto DeviceManager::get_distance(geometry_msgs::Pose const &a,
                            geometry_msgs::Pose const &b) -> std::pair<double, double>
{
  if (motion_)
    return motion_->get_distance(a, b);
  return std::make_pair(0.0, 0.0);
}

auto DeviceManager::get_nearest_info(nav_msgs::Path const &path, size_t const &index, int lb, int rb, bool following)
    -> std::pair<size_t, double>
{
  if (motion_)
    return motion_->get_nearest_info(path, index, lb, rb, following);
  return std::make_pair(index, 0.0);
}

void DeviceManager::point_cb(geometry_msgs::PointStampedConstPtr const &msg)
{
  if (motion_)
    motion_->point_cb(msg);
}

void DeviceManager::costmap_cb(nav_msgs::OccupancyGrid::ConstPtr const &msg)
{
  std::lock_guard<std::mutex> lock(map_update_mutex_);
  if (!costmap_)
  {
    ROS_WARN("sub costmap, init map OK!");
    costmap_ = std::make_shared<costmap_2d::Costmap2D>(msg->info.width, msg->info.height, msg->info.resolution,
                                                       msg->info.origin.position.x, msg->info.origin.position.y);
  }
  else
  {
    ROS_WARN("Update costmap!");
    costmap_->resizeMap(msg->info.width, msg->info.height, msg->info.resolution,
                        msg->info.origin.position.x, msg->info.origin.position.y);
  }

  uint32_t x;
  for (uint32_t y = 0; y < msg->info.height; y++)
  {
    for (x = 0; x < msg->info.width; x++)
    {
      if (msg->data[y * msg->info.width + x] < 0)
      {
        costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
        continue;
      }
      costmap_->setCost(x, y, cost_translation_[msg->data[y * msg->info.width + x]]);
    }
  }
}

void DeviceManager::costmap_update_cb(map_msgs::OccupancyGridUpdate::ConstPtr const &msg)
{
  std::lock_guard<std::mutex> lock(map_update_mutex_);
  if (!costmap_)
  {
    ROS_WARN("Costmap not initiate yet");
    return;
  }

  if (msg->width * msg->height != msg->data.size() || msg->x + msg->width > costmap_->getSizeInCellsX() || msg->y + msg->height > costmap_->getSizeInCellsY())
  {
    ROS_ERROR("Costmap update got invalid data set");
    return;
  }

  size_t index = 0;
  int x;
  for (auto y = msg->y; y < msg->y + msg->height; y++)
  {
    for (x = msg->x; x < msg->x + msg->width; x++)
    {
      if (msg->data[index] < 0)
      {
        costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
        index++;
        continue;
      }
      costmap_->setCost(x, y, cost_translation_[msg->data[index++]]);
    }
  }
}

auto DeviceManager::is_free(const geometry_msgs::PoseStamped &pose) const -> bool
{
  auto cost = pose_cost(pose);
  return cost < 66;
}

auto DeviceManager::is_danger(const geometry_msgs::PoseStamped &pose) const -> bool
{
  auto cost = pose_cost(pose);
  return cost >= 253;
}

auto DeviceManager::pose_cost(const geometry_msgs::PoseStamped &pose) const -> uint8_t
{
  if (!costmap_)
  {
    // HACK
    // ROS_WARN("No costmap yet");
    return true;
  }

  uint32_t mx, my;
  if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
  {
    ROS_WARN("Can't find point mx, my in cost map");
    return 0;
  }

  return costmap_->getCost(mx, my);
}

void DeviceManager::goto_done(const actionlib::SimpleClientGoalState &state,
                             const mbf_msgs::MoveBaseResultConstPtr &result)
{
  ROS_INFO("MoveBase got state [%s]", state.toString().c_str());

  if (!result)
    return;
  ROS_INFO("MoveBase got result [%d]", result->outcome);
  //  send_exe(current_index_);
}

auto DeviceManager::send_exe(size_t const &index) -> bool
{
  if (motion_)
    return motion_->send_exe(index);
  return false;
}

void DeviceManager::exe_done(const actionlib::SimpleClientGoalState &state,
                            const mbf_msgs::ExePathResultConstPtr &result)
{
  ROS_INFO("ExePath got state [%s]", state.toString().c_str());

  if (!result)
    return;
  ROS_INFO("ExePath got result [%d]", result->outcome);
  
  // 路径执行完成后，转换到ExecuteAction状态
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED && 
      task_state_ == TaskStatus::GoForwardRoute)
  {
    ROS_INFO("Path execution completed, transitioning to ExecuteAction");
    task_state_ = TaskStatus::ExecuteAction;
  }
}

auto DeviceManager::check_file_exist(std::string &file_path) -> bool
{
  std::ifstream exist(file_path.c_str());
  return !!exist;
}

auto DeviceManager::write_file(std::string &file_path, std::vector<nav_msgs::Path> const &paths) -> bool
{
  std::ofstream out(file_path.c_str());
  if (!out.is_open())
  {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  for (auto const &path : paths)
  {
    for (auto const &p : path.poses)
    {
      out << std::to_string(p.pose.position.x) << " "
          << std::to_string(p.pose.position.y) << " "
          << std::to_string(tf2::getYaw(p.pose.orientation)) << "\n";
    }
    out << "EOP" << "\n";
  }

  out.close();
  return true;
}

auto DeviceManager::read_file(std::string &file_path) -> bool
{
  std::ifstream in(file_path.c_str());
  if (!in.is_open())
  {
    ROS_ERROR("Open file %s failed!", file_path.c_str());
    return false;
  }

  nav_msgs::Path route;
  route.header.frame_id = "map";
  route.header.stamp = ros::Time::now();
  std::string contend, temp;
  std::vector<std::string> temps;
  record_path_.poses.clear();
  while (getline(in, contend))
  {
    if (contend == "EOP" && !route.poses.empty())
    {
      ROS_INFO("Route got %lu poses", route.poses.size());
      motion_->AppendExeRoute(route);
      route.poses.clear();
      continue;
    }

    temps.clear();
    temp.clear();
    for (auto const &c : contend)
    {
      if (c != ' ')
      {
        temp += c;
      }
      else
      {
        temps.emplace_back(temp);
        temp.clear();
      }
    }
    if (!temp.empty())
      temps.emplace_back(temp);
    if (temps.size() != 3)
      continue;
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = std::stod(temps[0]);
    p.pose.position.y = std::stod(temps[1]);
    p.pose.orientation.z = std::sin(std::stod(temps[2]) / 2.0);
    p.pose.orientation.w = std::cos(std::stod(temps[2]) / 2.0);
    route.poses.emplace_back(p);
    record_path_.poses.emplace_back(p);
  }
  record_path_.poses.clear();
  RecordPathWaypoint();
  // if motion_ exists, assume routes were appended there
  return true;
}
