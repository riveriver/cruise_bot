#include <csignal>

#include "DeviceManager.hpp"

#define DECISION_VERSION "2.0.0"
std::shared_ptr<DeviceManager> manager_ptr;

void sigintHandler(int sig) {
  if (manager_ptr) {
    ROS_ERROR("[sigintHandler] stop robot manager");
    manager_ptr->pause_move();
    manager_ptr->stop_move();
  }
  ros::shutdown();
}

int main(int argc, char** argv) {
  
  // 初始化区域设置，支持中文输出
  setlocale(LC_ALL, "");
  
  // 初始化ROS节点
  ros::init(argc, argv,"DeviceManager");
  // 设置日志级别为Info
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  // 创建节点句柄
  ros::NodeHandle nh;

  // 创建RobotManager智能指针
  manager_ptr = std::make_shared<DeviceManager>(nh);
  ROS_INFO("device_manager_ptr init success! version: %s", DECISION_VERSION);

  // 设置信号处理函数
  signal(SIGINT, sigintHandler);

  // 初始化RobotManager
  manager_ptr->init();
  
  // 运行RobotManager
  manager_ptr->run();
  
  return 0;
}
