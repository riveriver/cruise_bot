#pragma once
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <utility>
#include "DeviceManagerCommon.hpp"

class DeviceManager;

class DeviceManagerMotion
{
public:
  explicit DeviceManagerMotion(DeviceManager *owner);
  // initialize publishers/subscribers/action clients
  void init();
  void ExecuteMoveState();
  auto send_goto(size_t const &index) -> bool;
  auto send_exe(size_t const &index) -> bool;
  void pub_zero_vel();
  void pause_move();
  void stop_move();
  void resume_move();
  auto get_distance(geometry_msgs::Pose const &a, geometry_msgs::Pose const &b) -> std::pair<double, double>;
  auto get_nearest_info(nav_msgs::Path const &path, size_t const &index, int lb, int rb, bool following)
      -> std::pair<size_t, double>;
  void point_cb(geometry_msgs::PointStampedConstPtr const &msg);

  // helper to publish record path from Task code
  void publishRecordPath(const nav_msgs::Path &p);
  // append route parsed from file into internal exe_routes
  void AppendExeRoute(const nav_msgs::Path &route);
  // pop next exe_route and start execution (publish record, set move state and send goto)
  bool StartExecuteFromFront();

private:
  DeviceManager *owner_;
  // motion-owned resources
  std::unique_ptr<GotoCtrl> goto_ctrl_;
  std::unique_ptr<ExeCtrl> exe_ctrl_;
  std::list<nav_msgs::Path> exe_routes_;
  nav_msgs::Path cur_route_;
  size_t cur_index_;
  size_t obs_index_;
  MoveStatus move_state_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher cur_pose_pub_;
  ros::Publisher record_point_pub_;
  ros::Publisher record_path_pub_;
  std::vector<geometry_msgs::PointStamped> record_points_;
  ros::Subscriber point_sub_;
  ros::Subscriber costmap_sub_;        // optional, if motion needs it
  ros::Subscriber costmap_update_sub_; // optional
};
