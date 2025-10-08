#include "DeviceManager.hpp"
#include "DeviceManagerMotion.hpp"
#include <algorithm>

DeviceManagerMotion::DeviceManagerMotion(DeviceManager *owner) : owner_(owner), cur_index_(0), obs_index_(0), move_state_(MoveStatus::GoGoal), is_reverse_execution_(false) {}

void DeviceManagerMotion::init()
{
  // create action clients owned by motion
  goto_ctrl_.reset(new GotoCtrl("move_base_flex/move_base"));
  exe_ctrl_.reset(new ExeCtrl("move_base_flex/exe_path"));

  // wait for servers
  ROS_ERROR_COND(!goto_ctrl_->waitForServer(ros::Duration(5.0)), "move base action not online!");
  ROS_ERROR_COND(!exe_ctrl_->waitForServer(ros::Duration(5.0)), "exe path action not online!");

  // publishers
  ros::NodeHandle nh;
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  record_path_pub_ = nh.advertise<nav_msgs::Path>("record_path", 1);
  record_point_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("record_zone", 1);
  cur_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1);

  // subscribers
  point_sub_ = nh.subscribe("/clicked_point", 1, &DeviceManagerMotion::point_cb, this);
  // costmap subscribers left to DeviceManager if needed
}

void DeviceManagerMotion::ExecuteMoveBehavior()
{
  auto *dm = owner_;
  using goalState = actionlib::SimpleClientGoalState;
  static int wait_cnt = 0;
  switch (move_state_)
  {
  // 跟线状态
  case MoveStatus::ExePath:
  {
    if (exe_ctrl_->getState() == goalState::SUCCEEDED)
    {
      if (exe_routes_.empty())
      {
        stop_move();
        // 根据是否为反向执行设置不同的状态
        if (is_reverse_execution_) {
          dm->task_state_ = TaskStatus::CompletedRoute;
          is_reverse_execution_ = false;  // 重置标记
          ROS_INFO("Reverse route completed! Enter Completed Route.");
        } else {
          dm->task_state_ = TaskStatus::ExecuteAction;
          ROS_INFO("Forward route success! Enter Execute Action");
        }
      }
      else
      {
        cur_route_ = exe_routes_.front();
        exe_routes_.pop_front();
        cur_index_ = 0;
        move_state_ = MoveStatus::GoGoal;
        // delegate to DeviceManager for goto_done callback via send_goto
        owner_->send_goto(cur_index_);
      }
      return;
    }

    if (exe_ctrl_->getState().isDone())
    {
      for (auto i = cur_index_; i < cur_route_.poses.size(); ++i)
      {
        if (owner_->is_free(cur_route_.poses.at(i)))
        {
          cur_index_ = i;
          break;
        }
      }

      cur_index_ = std::min(cur_index_, cur_route_.poses.size() - 1);
      cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));
      ROS_INFO("Exe path failed, start goto, target %lu", cur_index_);
      move_state_ = MoveStatus::GoGoal;
      send_goto(cur_index_);
    }
    cur_index_ = get_nearest_info(cur_route_, cur_index_, 0, 20, true).first;
    cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));

    for (auto i = 0; i < PATH_SAFE_DIS_NUM; ++i)
    {
      if (cur_index_ + i >= cur_route_.poses.size())
        break;
      if (owner_->is_danger(cur_route_.poses.at(cur_index_ + i)))
      {
        obs_index_ = cur_index_ + i;
        ROS_WARN("Obs found at %.3f away",
                 get_distance(cur_route_.poses[cur_index_].pose, cur_route_.poses[obs_index_].pose).first);
        pause_move();
        wait_cnt = 0;
        move_state_ = MoveStatus::WaitSafe;
      }
    }
    break;
  }

  // 避障等待状态
  case MoveStatus::WaitSafe:
  {
    auto still_danger = false;
    for (auto i = 0; i < 10; ++i)
    {
      if (obs_index_ + i >= cur_route_.poses.size())
        break;
      if (owner_->is_danger(cur_route_.poses.at(obs_index_ + i)))
      {
        still_danger = true;
        break;
      }
    }

    if (!still_danger)
    {
      ROS_INFO("Obs clear, keep moving");
      move_state_ = MoveStatus::ExePath;
      send_exe(cur_index_);
      break;
    }

    if (wait_cnt++ > WAIT_COUNT)
    {
      for (auto i = obs_index_; i < cur_route_.poses.size(); ++i)
      {
        if (owner_->is_free(cur_route_.poses.at(i)))
        {
          cur_index_ = i;
          break;
        }
      }
      cur_index_ = std::min(cur_index_, cur_route_.poses.size() - 1);
      cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));
      ROS_INFO("Wait finish, start avoid, target %lu", cur_index_);
      move_state_ = MoveStatus::GoGoal;
      send_goto(cur_index_);
    }
    break;
  }

  // 点到点状态
  case MoveStatus::GoGoal:
  {
    if (goto_ctrl_->getState() == goalState::SUCCEEDED
        /*&& get_distance(robot_pose().value(), cur_route_.poses[cur_index_].pose).first < 0.1
        && get_distance(robot_pose().value(), cur_route_.poses[cur_index_].pose).second < 10 * DEG2RAD*/
    )
    {
      ROS_INFO("MoveBase done, start exe path");
      move_state_ = MoveStatus::ExePath;
      send_exe(cur_index_);
      break;
    }


    if (goto_ctrl_->getState().isDone() || !owner_->is_free(cur_route_.poses.at(cur_index_)))
    {
      for (auto i = cur_index_ + 10; i < cur_route_.poses.size(); ++i)
      {
        if (owner_->is_free(cur_route_.poses.at(i)))
        {
          cur_index_ = i;
          break;
        }
      }

      if (cur_index_ >= cur_route_.poses.size())
      {
        if (exe_routes_.empty())
        {
          ROS_WARN("Task failed!");
          stop_move();
        }
        else
        {
          cur_route_ = exe_routes_.front();
          cur_index_ = 0;
          exe_routes_.pop_front();
          send_goto(cur_index_);
        }
        return;
      }

      cur_pose_pub_.publish(cur_route_.poses.at(cur_index_));
      ROS_WARN("Target not safe, update %lu", cur_index_);
      send_goto(cur_index_);
    }
    break;
  }
  
  default:
    ROS_ERROR("Error MoveStatus!");
    break;
  }
}

auto DeviceManagerMotion::send_goto(size_t const &index) -> bool
{
  auto *dm = owner_;
  // 检查索引是否有效
  if (index == std::numeric_limits<size_t>::max() || index > cur_route_.poses.size())
  {
    ROS_ERROR_STREAM("send_goto index error " << index << " / " << cur_route_.poses.size());
    return false;
  }

  mbf_msgs::MoveBaseGoal goal{};
  goal.target_pose = cur_route_.poses.at(index);
  ROS_INFO("send_goto goal");
  goto_ctrl_->sendGoal(goal,
                       boost::bind(&DeviceManager::goto_done, owner_, _1, _2),
                       GotoCtrl::SimpleActiveCallback(),
                       GotoCtrl::SimpleFeedbackCallback());
  return true;
}

auto DeviceManagerMotion::send_exe(size_t const &index) -> bool
{
  if (index == std::numeric_limits<size_t>::max() || index > cur_route_.poses.size())
  {
    ROS_ERROR_STREAM("send_exe index error " << index << " / " << cur_route_.poses.size());
    return false;
  }

  mbf_msgs::ExePathGoal goal{};
  nav_msgs::Path route;
  route.header = cur_route_.header;
  route.poses.assign(cur_route_.poses.begin() + index, cur_route_.poses.end());
  goal.path = route;
  ROS_INFO("send_exe goal");
  exe_ctrl_->sendGoal(goal,
                      boost::bind(&DeviceManager::exe_done, owner_, _1, _2),
                      ExeCtrl::SimpleActiveCallback(),
                      ExeCtrl::SimpleFeedbackCallback());
  return true;
}

void DeviceManagerMotion::pub_zero_vel()
{
  auto *dm = owner_;
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.angular.z = 0;
  cmd_vel_pub_.publish(vel);
}

void DeviceManagerMotion::pause_move()
{
  if (!goto_ctrl_->getState().isDone())
  {
    ROS_INFO("Cancel current goto goal");
    goto_ctrl_->cancelGoal();
  }

  if (!exe_ctrl_->getState().isDone())
  {
    ROS_INFO("Cancel current exe path goal");
    exe_ctrl_->cancelGoal();
  }

  for (auto i = 0; i < 10; ++i)
  {
    pub_zero_vel();
    ros::Duration(0.1).sleep();
  }
}

void DeviceManagerMotion::stop_move()
{
  move_state_ = MoveStatus::GoGoal;
  cur_index_ = 0;
  cur_route_.poses.clear();
  exe_routes_.clear();
  pause_move();
}

void DeviceManagerMotion::resume_move()
{
  auto nearest = owner_->get_nearest_info(cur_route_, cur_index_, 0, 20, false);
  cur_index_ = nearest.second < 0.1 ? nearest.first : cur_index_;
  move_state_ = MoveStatus::GoGoal;
  send_goto(cur_index_);
}

auto DeviceManagerMotion::get_distance(geometry_msgs::Pose const &a,
                            geometry_msgs::Pose const &b) -> std::pair<double, double>
{
  std::pair<double, double> result;
  result.first = std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
  result.second = std::abs(tf::getYaw(a.orientation) - tf::getYaw(b.orientation));
  return result;
}

auto DeviceManagerMotion::get_nearest_info(nav_msgs::Path const &path, size_t const &index, int lb, int rb, bool following)
    -> std::pair<size_t, double>
{
  DeviceManager *dm = owner_;
  size_t idx = index;
  auto robot_pos = dm->robot_pose();
  if (!robot_pos)
    return std::make_pair(index, 0.0);

  auto dis = std::numeric_limits<double>::max();
  lb = std::max(static_cast<int>(index) - lb, 0);
  rb = std::min(index + rb, path.poses.size());
  if (following && get_distance(robot_pos.value(), path.poses.at(index).pose).first > 1.0)
  {
    rb = path.poses.size();
  }

  for (auto i = lb; i < rb; ++i)
  {
    auto err = get_distance(robot_pos.value(), path.poses[i].pose);
    if (following)
      err.second = 0;
    if (err.first < dis && err.second < 30 * DEG2RAD)
    {
      dis = err.first;
      idx = static_cast<size_t>(i);
    }
  }

  return std::make_pair(idx, dis);
}

void DeviceManagerMotion::point_cb(geometry_msgs::PointStampedConstPtr const &msg)
{
  if (owner_->system_mode_ == SystemMode::RecordRoute)
    return;
  record_points_.emplace_back(*msg);
  geometry_msgs::PolygonStamped polygon;
  polygon.header.frame_id = "map";
  polygon.header.stamp = ros::Time::now();
  geometry_msgs::Point32 point32;
  for (auto const &p : record_points_)
  {
    point32.x = p.point.x;
    point32.y = p.point.y;
    point32.z = 0;
    polygon.polygon.points.emplace_back(point32);
  }
  record_point_pub_.publish(polygon);
}

void DeviceManagerMotion::AppendExeRoute(const nav_msgs::Path &route)
{
  exe_routes_.push_back(route);
}

bool DeviceManagerMotion::StartExecuteFromFront()
{
  if (exe_routes_.empty())
    return false;
  cur_route_ = exe_routes_.front();
  exe_routes_.pop_front();
  record_path_pub_.publish(cur_route_);
  move_state_ = MoveStatus::GoGoal;
  cur_index_ = 0;
  is_reverse_execution_ = false;  // 设置为正向执行
  send_goto(cur_index_);
  return true;
}

bool DeviceManagerMotion::StartExecuteReverse()
{
  if (cur_route_.poses.empty())
    return false;
  
  // Create a reversed copy of the current route
  nav_msgs::Path reversed_route = cur_route_;
  std::reverse(reversed_route.poses.begin(), reversed_route.poses.end());
  
  // Update timestamps
  reversed_route.header.stamp = ros::Time::now();
  for (auto& pose : reversed_route.poses) {
    pose.header.stamp = ros::Time::now();
  }
  
  // Set as current route and start execution
  cur_route_ = reversed_route;
  record_path_pub_.publish(cur_route_);
  move_state_ = MoveStatus::GoGoal;
  cur_index_ = 0;
  is_reverse_execution_ = true;   // 设置为反向执行
  send_goto(cur_index_);
  return true;
}

void DeviceManagerMotion::publishRecordPath(const nav_msgs::Path &p)
{
  if (record_path_pub_)
    record_path_pub_.publish(p);
}
// end of DeviceManagerMotion.cpp
// end of DeviceManagerMotion.cpp
