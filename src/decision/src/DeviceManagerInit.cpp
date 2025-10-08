#include "DeviceManager.hpp"
#include "DeviceManagerMqtt.hpp"
#include "DeviceManagerMotion.hpp"
#include "DeviceManagerTask.hpp"
#include <ros/package.h>

// ...existing code...

DeviceManager::DeviceManager(ros::NodeHandle &nh)
    : nh_(nh),
      system_mode_(SystemMode::AutoNav),
      task_state_(TaskStatus::PubTask),
      building_service_(new TaskExecuter("task_executer/building_service")),
      air_quality_(new TaskExecuter("task_executer/air_quality")),
      lio_relocate_(new TaskExecuter("task_executer/lio_relocate")),
      battery_level_(80),
      charge_state_(false),
      is_app_control(false),
      is_live_stream(false),
      is_iaq_upload(false),
      air_quality_goal_sent_(false),  // 初始化air_quality Goal发送标志位
      
      tf_robot_pose(new tf2_ros::Buffer()),
      tf_pose_listener(new tf2_ros::TransformListener(*tf_robot_pose))
{
  resource_path_ = ros::package::getPath("cruise_decision");
  record_path_.header.frame_id = "map";
  if (!cost_translation_)
  {
    cost_translation_ = new uint8_t[101];
    for (int i = 0; i < 101; ++i)
    {
      cost_translation_[i] = static_cast<uint8_t>(i * 254 / 100);
    }
  }

  // Instantiate helpers
  motion_ = std::make_unique<DeviceManagerMotion>(this);
  task_helper_ = std::make_unique<DeviceManagerTask>(this);
}

DeviceManager::~DeviceManager()
{
  // 清理 MQTT 客户端与定时器
  if (mqtt_client_)
  {
    mqtt_client_->destroy();
    mqtt_client_.reset();
  }
  motion_.reset();
  task_helper_.reset();
  upload_info_timer_.stop();
  upload_fast_info_timer_.stop();
  upload_sensor_timer_.stop();
  ROS_INFO("~DeviceManager");
}

void DeviceManager::init()
{
  ros::NodeHandle nh;

  // 初始化定时器
  upload_fast_info_timer_ = nh_.createTimer(ros::Duration(1.0), &DeviceManager::UploadFastInfoCallback, this);
  upload_sensor_timer_ = nh_.createTimer(ros::Duration(1.0), &DeviceManager::UploadAirCallback, this);
  upload_sensor_timer_.stop();

  // Device-level subscribers/publishers remain. Motion-specific init delegated to motion_->init().
  base_battery_sub_ = nh_.subscribe("/battery_state", 10, &DeviceManager::SubBatteryCallback, this);
  air_data_sub_ = nh_.subscribe("/sensors/air_data", 10, &DeviceManager::SubAirDataCallback, this);
  air_result_sub_ = nh_.subscribe("/sensors/air_result", 10, &DeviceManager::SubAirResultCallback, this);
  device_pose_sub_ = nh_.subscribe("/base_pose_ground_truth", 10, &DeviceManager::SubDevicePoseCallback, this);

  // 初始化 Service
  task_cmd_ = nh.advertiseService("task_cmd", &DeviceManager::TaskCmdCallback, this);

  // motion helper will initialize action clients and wait for servers
  if (motion_)
    motion_->init();

  // 初始化 MQTT 接口（实现位于 DeviceManagerMqtt.cpp）
  mqtt_client_.reset(new DeviceManagerMqtt());
  int8_t mqtt_error = mqtt_client_->init();
  if (mqtt_error != 0)
  {
    ROS_ERROR("Failed to initialize MQTT interface, error code: %d", mqtt_error);
  }
  else
  {
    // 订阅 topic 并将回调绑定到当前 DeviceManager 实例的方法
    mqtt_client_->subscribe("device/prototypes001/command", std::bind(&DeviceManager::mqtt2rosCmdCallback, this, std::placeholders::_1));
    mqtt_client_->subscribe("device/prototypes001/move", std::bind(&DeviceManager::mqtt2rosMoveCallback, this, std::placeholders::_1));
    mqtt_client_->subscribe("device/prototypes001/info", std::bind(&DeviceManager::mqtt2rosInfoCallback, this, std::placeholders::_1));
    mqtt_client_->subscribe("device/prototypes001/task", std::bind(&DeviceManager::mqtt2rosTaskCallback, this, std::placeholders::_1));
    mqtt_client_->subscribe("device/prototypes001/sensor", std::bind(&DeviceManager::mqtt2rosSensorCallback, this, std::placeholders::_1));
    ROS_INFO("mqtt init success");
  }
  ROS_INFO("hao sen gen niu bi! DeviceManager init success!");
}
