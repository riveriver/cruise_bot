#include <string>
#include <signal.h>
#include <ros/ros.h>
#include "BehaviorActionServer.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "iaq_action_server_node");
    BehaviorActionServer iaq_action_server("/iaq_action_server");
    return 0;
}