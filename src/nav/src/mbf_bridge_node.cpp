//
// Created by river on 2025/05/06.
//
// 本节点作为move_base_flex的桥接器，实现如下功能：
// 1. 接收move_base_simple/goal的PoseStamped消息，封装为MoveBaseActionGoal并转发到move_base_flex/move_base/goal。
// 2. 提供xju_zone服务，分别调用local/global costmap的keep_out_zone服务，实现禁入区的同步设置。

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#include "mbf_msgs/MoveBaseAction.h"

namespace xju::pnc {
// MbfBridge类：实现move_base_simple与move_base_flex的桥接，以及禁入区服务同步
class MbfBridge {
public:
  
  // 定时器回调函数，实际执行costmap清理
  void clearCostmapsTimerCallback(const ros::TimerEvent&) {
    if(!clear_costmaps_client_.exists()) {
      ROS_WARN("Clear costmaps service not available");
      return;
    }

    std_srvs::Empty srv;
    if(clear_costmaps_client_.call(srv)) {
      ROS_INFO("Costmaps cleared after initialpose update");
    } else {
      ROS_ERROR("Failed to clear costmaps");
    }
  }

  // initialpose回调函数，启动3秒后清理costmap的定时器
  void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    // 最后一个参数oneshot为true，表示定时器是单次触发
    clear_costmaps_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                          &MbfBridge::clearCostmapsTimerCallback, 
                                          this, true);
    clear_costmaps_timer_.start();
    ROS_INFO("Scheduled costmap clearing in 3 seconds");
  }

  // 构造函数，初始化发布、订阅和服务
  MbfBridge(){

    ros::NodeHandle nh_;  // ROS节点句柄
    // 订阅initialpose话题
    initialpose_sub_ = nh_.subscribe("/initialpose", 1, &MbfBridge::initialposeCallback, this);
    clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base_flex/clear_costmaps");

    // 在ROS中，topic的全名（fully qualified name）由命名空间（namespace）和topic名共同决定。
    // 故在这里，全名是/move_base_flex/move_base/goal
    ros::NodeHandle action_nh("move_base_flex/move_base");
    action_goal_pub_ = action_nh.advertise<mbf_msgs::MoveBaseActionGoal>("goal", 1);
    ROS_INFO("action_goal_pub_ is ready.");
    // 故在这里，全名是/move_base_simple/goal
    // 订阅move_base_simple/goal，回调goalCB
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MbfBridge::goalCB, this, _1));
    ROS_INFO("goal_sub_ is ready.");
  }

  ~MbfBridge() = default;

private:
  // goal话题回调，将PoseStamped封装为MoveBaseActionGoal并发布
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base_flex","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    ROS_INFO("MbfBridge::goalCB");
    mbf_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

private:
  ros::Publisher action_goal_pub_;      // 发布MoveBaseActionGoal的发布器
  ros::Subscriber goal_sub_;            // 订阅PoseStamped goal的订阅器
  ros::Subscriber initialpose_sub_ ;
  ros::ServiceClient clear_costmaps_client_;
  ros::Timer clear_costmaps_timer_;     // 用于延迟清理costmap的定时器
  ros::NodeHandle nh_;                  // 节点句柄

};
}

// 主函数，初始化ROS节点并spin
int main(int argc, char** argv) {
  ros::init(argc, argv, "mbf_bridge");

  xju::pnc::MbfBridge _bridge;
  ros::spin();

  return 0;
}
