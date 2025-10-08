#pragma once
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>

using GotoCtrl = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;
using ExeCtrl = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;

enum class MoveStatus : uint8_t
{
  GoGoal = 0,
  ExePath,
  WaitSafe
};
