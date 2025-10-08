#pragma once
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "DeviceManagerCommon.hpp"

#include <vector>
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"
// C++
#include <vector>
#include <ctime>
#include <mutex>
#include <deque>
#include <string>
#include <map>
#include <iomanip>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "ranger_msgs/BatteryState.h"
#include "ranger_msgs/ActuatorState.h"
#include "ranger_msgs/DriverState.h"
#include "ranger_msgs/MotorState.h"
#include "ranger_msgs/MotionState.h"
#include "ranger_msgs/SystemState.h"
#include "ranger_msgs/TriggerParkMode.h"
#include "cruise_msgs/AirSensors.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include "cruise_msgs/TaskExecuterAction.h"
#include "cruise_msgs/TaskService.h"
/* Function */
#include <jsoncpp/json/json.h>
#include <sound_play/sound_play.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

constexpr static const double DEG2RAD = M_PI / 180;
constexpr static const double RECORD_PATH_LEN_DENS = 0.05;
constexpr static const double RECORD_PATH_AGU_DENS = 10 * DEG2RAD;
constexpr static const int WAIT_COUNT = 5 * 10;                            // 5s
constexpr static const int PATH_SAFE_DIS_NUM = 1.3 / RECORD_PATH_LEN_DENS; // 1.3m

enum class SystemStatus : uint8_t
{
    Idle = 0,
    Record,
    AppControlMode,
    Run,
    Pause
};

// MoveStatus defined in DeviceManagerCommon.hpp

    struct DevicePoseDefine
    {
        float x;
        float y;
        float yaw;
    };

    struct AreaPoseDefine
    {
        int floor;
        int num;
    };

    struct SpotPoseDefine
    {
        int floor;
        int x;
        int y;
    };

    struct TaskInfo
    {
        std::string id = "0";
        std::time_t set_time;
        std::time_t start_time;
        std::string project;
        int progress;
        std::string state;
        std::vector<std::string> actions;
        std::string pose_mode;
        int8_t pose_index = -1;
        int8_t action_index = 0;
        std::vector<AreaPoseDefine> areas;
        std::vector<SpotPoseDefine> spots;
    };

    struct MqttMsgHead
    {
        std::string topic;
        std::string type;
        std::string id;
        std::string msg;
        int code;
    int qos = 0;
    };

    struct AppMoveCmd
    {
        int speed;
        bool stop;
        bool up;
        bool down;
        bool left;
        bool right;
        bool rotate_left;
        bool rotate_right;
        int rev_time;
    };

    enum class TaskStatus : uint8_t
    {
        PubTask = 0,
        PubRoute,
        GoForwardRoute,
        ExecuteAction,
        PubReverseRoute,    // 发布反向路线
        GoReverseRoute,  // 执行反向路线
        CompletedRoute       // 任务完成，准备进入下一个任务
    };

    enum class SystemMode : uint8_t
    {
        Idle = 0,
        AppControl,
        RecordRoute,
        AutoNav,
        BackHome
    };

// GotoCtrl/ExeCtrl typedefs are defined in DeviceManagerCommon.hpp
using TaskExecuter = actionlib::SimpleActionClient<cruise_msgs::TaskExecuterAction>;

class DeviceManager
{
public:
  // DeviceManager();
  explicit DeviceManager(ros::NodeHandle &nh);
  ~DeviceManager();

  void init();

  void run();

  void pause_move();
  void resume_move();
  void stop_move();

private:
  void ExecuteMoveBehavior();

  void pub_zero_vel();


  auto get_distance(geometry_msgs::Pose const &a, geometry_msgs::Pose const &b) -> std::pair<double, double>;

  auto get_nearest_info(nav_msgs::Path const &path,
                    size_t const &index,
                    int lb = std::numeric_limits<int>::max(),
                    int rb = std::numeric_limits<int>::max(),
                    bool following = false) -> std::pair<size_t, double>;

  void costmap_cb(nav_msgs::OccupancyGrid::ConstPtr const &msg);

  void costmap_update_cb(map_msgs::OccupancyGridUpdate::ConstPtr const &msg);

  void point_cb(geometry_msgs::PointStampedConstPtr const &msg);

  auto is_free(const geometry_msgs::PoseStamped &pose) const -> bool;

  auto is_danger(const geometry_msgs::PoseStamped &pose) const -> bool;

  auto pose_cost(const geometry_msgs::PoseStamped &pose) const -> uint8_t;

  auto send_goto(size_t const &index) -> bool;

  void goto_done(const actionlib::SimpleClientGoalState &state, const mbf_msgs::MoveBaseResultConstPtr &result);

  auto send_exe(size_t const &index) -> bool;

  void exe_done(const actionlib::SimpleClientGoalState &state, const mbf_msgs::ExePathResultConstPtr &result);

  auto TaskCmdCallback(cruise_msgs::TaskService::Request &req, cruise_msgs::TaskService::Response &resp) -> bool;

  void record_path();


  void AppControlBehavior();
  void RecordRouteBehavior();
  void AutoNavBehavior();
  void UpdateSystemMode(SystemMode mode);
  bool ExecuteActionBehavior();
  bool PubReverseRoute();
  void HandleRouteCompleted();
  void start_reverse_execution();
  // void CaptureDoneCallback(const cruise_msgs::CaptureServiceResponse& res);

private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher cur_pose_pub_;
  ros::Publisher normal_goal_pub_;
  ros::Publisher record_point_pub_;
  ros::Subscriber point_sub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_update_sub_;
  ros::Subscriber traffic_goal_sub_;
    // motion control moved into DeviceManagerMotion
  std::unique_ptr<TaskExecuter> building_service_;
  std::unique_ptr<TaskExecuter> air_quality_;
  std::unique_ptr<TaskExecuter> lio_relocate_;
  ros::ServiceClient wakeup_theta_client_;
  ros::ServiceClient start_theta_client_;
  void BuildingServiceDone(const actionlib::SimpleClientGoalState &state,
                           const cruise_msgs::TaskExecuterResultConstPtr &result);
  void AirQualityDone(const actionlib::SimpleClientGoalState &state,
                      const cruise_msgs::TaskExecuterResultConstPtr &result);

      std::string file_path_;

    std::vector<geometry_msgs::PointStamped> record_points_;

    size_t obs_index_;
    
    double left_d_, right_d_;

    std::mutex map_update_mutex_;

    std::shared_ptr<costmap_2d::Costmap2D> costmap_;
    static uint8_t *cost_translation_;

    DevicePoseDefine bs_pose_;
    bool move_permit_;
    bool air_quality_goal_sent_;  // 标志位，记录是否已发送air_quality Goal

     /* System */
    std::string resource_path_;
    SystemMode system_mode_;
    TaskStatus task_state_;
    AppMoveCmd cmd_move;
    TaskInfo cur_task_;
    DevicePoseDefine device_pose_;
    nav_msgs::Path record_path_;
    std::unique_ptr<tf2_ros::Buffer> tf_robot_pose;
    std::unique_ptr<tf2_ros::TransformListener> tf_pose_listener;
    std::unique_ptr<GotoCtrl> goto_ctrl_;
    // auto TaskCmdCallback(cruise_msgs::TaskService::Request &req, cruise_msgs::TaskService::Response &resp) -> bool;
    bool PubTask();
    bool PubRoute();
    void CheckReadyTask();
    void RecordPathWaypoint();
    auto robot_pose() -> std::optional<geometry_msgs::Pose>;

    // helpers
    std::unique_ptr<class DeviceManagerMotion> motion_;
    std::unique_ptr<class DeviceManagerTask> task_helper_;

    // allow helper classes to access private members for delegation
    friend class DeviceManagerMotion;
    friend class DeviceManagerTask;

    

private:
    /* Data */
    float battery_level_;
    bool charge_state_;
    ros::Subscriber device_pose_sub_;
    ros::Subscriber base_battery_sub_;
    ros::Subscriber air_data_sub_;
    ros::Subscriber air_result_sub_;
    ros::ServiceServer task_cmd_;
    std::string TimeToYMDHMS(const std::time_t &time);
    void PackDeviceInfo(Json::Value &result);
    void PackDevicePose(Json::Value &result);
    void PackAirData(const cruise_msgs::AirSensors &data, Json::Value &result);
    void SubBatteryCallback(const ranger_msgs::BatteryState::ConstPtr &msg);
    void SubDevicePoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void SubAirDataCallback(const cruise_msgs::AirSensors::ConstPtr &msg);
    void SubAirResultCallback(const cruise_msgs::AirSensors::ConstPtr &msg);

    /* Comm */
    bool is_app_control;
    bool is_live_stream;
    bool is_iaq_upload;
    ros::Timer upload_info_timer_;
    ros::Timer upload_sensor_timer_;
    ros::Timer upload_fast_info_timer_;

    // MQTT 客户端封装
    std::unique_ptr<class DeviceManagerMqtt> mqtt_client_;

    void PubAckMsg(MqttMsgHead &msg_head, Json::Value payload);
    void mqtt2rosCmdCallback(const struct mosquitto_message *msg);
    void mqtt2rosInfoCallback(const struct mosquitto_message *msg);
    void mqtt2rosMoveCallback(const struct mosquitto_message *msg);
    void mqtt2rosTaskCallback(const struct mosquitto_message *msg);
    void mqtt2rosSensorCallback(const struct mosquitto_message *msg);
    void UploadAirData();
    void UploadSlowInfoCallback(const ros::TimerEvent &);
    void UploadFastInfoCallback(const ros::TimerEvent &);
    void UploadAirCallback(const ros::TimerEvent &);
    void SetAirUpload(bool enable);
    void SetLiveStream(bool enable);
    void SetControlMode(bool enable);
    void SetHomeMode(bool enable);
    void SetStopMode(bool enable);

    /* Task */
    uint8_t AddTasKInQueue(const TaskInfo &obj);
    uint8_t MoveTaskFrontQueue(const TaskInfo &obj);
    uint8_t RemoveTaskInQueue(const TaskInfo &obj);
    TaskInfo PeekFrontReadyTasks();
    TaskInfo PopFrontReadyTasks();
    std::string GetTaskInfoSimple();
    std::string GetTaskInfo();

    /* Module */
    cruise_msgs::AirSensors iaq_data;
    // uint8_t base_mode;

    auto check_file_exist(std::string &file_path) -> bool;
    auto write_file(std::string &file_path, std::vector<nav_msgs::Path> const &paths) -> bool;
    auto read_file(std::string &file_path) -> bool;
};
