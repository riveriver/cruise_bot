#include "DeviceManagerTask.hpp"
#include "DeviceManagerMotion.hpp"

DeviceManagerTask::DeviceManagerTask(DeviceManager *owner) : owner_(owner) {}

bool DeviceManagerTask::PubTask()
{
  auto *dm = owner_;
  if (!ready_tasks_.empty())
  {
    dm->cur_task_ = PeekFrontReadyTasks();
    dm->task_state_ = TaskStatus::PubRoute;
    
    // 重置air_quality Goal发送标志位，确保新任务可以正常发送Goal
    dm->air_quality_goal_sent_ = false;
    
    ROS_INFO("Start Task:%s", dm->cur_task_.id.c_str());
    ROS_INFO("actions:");
    for (const auto &action : dm->cur_task_.actions)
    {
      ROS_INFO(" - %s", action.c_str());
    }
    ROS_INFO("areas:");
    for (const auto &area : dm->cur_task_.areas)
    {
      ROS_INFO(" - Floor: %d, Num: %d", area.floor, area.num);
    }
    return true;
  }
  return false;
}

bool DeviceManagerTask::PubRoute()
{
  auto *dm = owner_;
  if (dm->cur_task_.pose_mode == "area")
  {
    // 切换到下一个位置
    dm->cur_task_.pose_index++;
    
    // 如果所有动作都执行完毕，则切换到下一个动作
    if (dm->cur_task_.pose_index >= dm->cur_task_.areas.size())
    {
      ROS_INFO("[PubRouteExecuter]route group Finished.Switch to next action.");
      dm->cur_task_.pose_index = 0;
      dm->cur_task_.action_index++;
    }

    // 如果所有动作都执行完毕，则切换到下一个任务
    if (dm->cur_task_.action_index >= dm->cur_task_.actions.size())
    {
      ROS_INFO("[PubRouteExecuter]action group Finished.Switch to next task.");
      dm->cur_task_.pose_index = -1;
      dm->cur_task_.action_index = 0;
      dm->PopFrontReadyTasks();
      dm->task_state_ = TaskStatus::PubTask;
      return false;
    }

    ROS_INFO("PubRouteExecuter:cur_task_.pose_index:%d, cur_task_.action_index:%d", dm->cur_task_.pose_index, dm->cur_task_.action_index);

    std::string route_name_ = "sim_" + std::to_string(dm->cur_task_.areas[dm->cur_task_.pose_index].floor) + "F_" +
                              std::to_string(dm->cur_task_.areas[dm->cur_task_.pose_index].num) + "A";
    std::string file_path_ = dm->resource_path_ + "/path/" + route_name_;
    if (!dm->check_file_exist(file_path_))
    {
      ROS_ERROR("file_path_ not exist:%s", file_path_.c_str());
      return false;
    }
    if (!dm->read_file(file_path_))
    {
      ROS_ERROR("Read path in file failed.");
      return false;
    }

    // append read routes are already in motion_. Use StartExecuteFromFront to start execution
    ROS_INFO("Pub Route:%s", route_name_.c_str());
    dm->motion_->StartExecuteFromFront();
    dm->task_state_ = TaskStatus::GoForwardRoute;  // 修复：应该先进入GoForwardRoute状态进行导航
    return true;
  }
  else
  {
    ROS_ERROR("PubRouteExecuter:Unknown Mode:%s", dm->cur_task_.pose_mode.c_str());
    return false;
  }
}

uint8_t DeviceManagerTask::AddTasKInQueue(const TaskInfo &obj)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  // prevent duplicates
  for (const auto &t : wait_tasks_)
  {
    if (t.id == obj.id)
      return 1; // already exists
  }
  wait_tasks_.push_back(obj);
  return 0;
}

uint8_t DeviceManagerTask::MoveTaskFrontQueue(const TaskInfo &obj)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  for (auto it = wait_tasks_.begin(); it != wait_tasks_.end(); ++it)
  {
    if (it->id == obj.id)
    {
      TaskInfo moved_task = std::move(*it);
      wait_tasks_.erase(it);
      moved_task.state = "front";
      wait_tasks_.push_front(std::move(moved_task));
      return 0;
    }
  }
  return 1;
}

uint8_t DeviceManagerTask::RemoveTaskInQueue(const TaskInfo &obj)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  for (auto it = wait_tasks_.begin(); it != wait_tasks_.end(); ++it)
  {
    if (it->id == obj.id)
    {
      wait_tasks_.erase(it);
      return 0;
    }
  }
  return 1;
}

TaskInfo DeviceManagerTask::PeekFrontReadyTasks()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (ready_tasks_.empty())
  {
    ROS_ERROR("peekFrontReadyTask:No tasks available");
    return {};
  }
  return ready_tasks_.front();
}

TaskInfo DeviceManagerTask::PopFrontReadyTasks()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (ready_tasks_.empty())
  {
    ROS_ERROR("getFrontReadyTask:No tasks available");
    return {};
  }
  TaskInfo front_task = ready_tasks_.front();
  ready_tasks_.pop_front();
  return front_task;
}

std::deque<TaskInfo> DeviceManagerTask::ListWaitTasks()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return wait_tasks_;
}

std::deque<TaskInfo> DeviceManagerTask::ListReadyTasks()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return ready_tasks_;
}

std::string DeviceManagerTask::GetTaskInfoSimple()
{
  std::ostringstream msg;
  bool first = true;
  msg << "WaitingTask:";
  for (const auto &task : wait_tasks_)
  {
    std::string taskIdLast5 = task.id.substr(std::max(static_cast<std::string::size_type>(0), task.id.size() - 5));
    if (!first) msg << ",";
    else first = false;
    msg << task.project << "_" << taskIdLast5;
  }
  msg << ";";
  msg << "ReadyTask:";
  first = true;
  for (const auto &task : ready_tasks_)
  {
    std::string taskIdLast5 = task.id.substr(std::max(static_cast<std::string::size_type>(0), task.id.size() - 5));
    if (!first) msg << ",";
    else first = false;
    msg << task.project << "_" << taskIdLast5;
  }
  return msg.str();
}

std::string DeviceManagerTask::GetTaskInfo()
{
  std::ostringstream msg;
  size_t maxProjectNameWidth = 10;
  for (const auto &task : wait_tasks_) maxProjectNameWidth = std::max(maxProjectNameWidth, task.project.size());
  for (const auto &task : ready_tasks_) maxProjectNameWidth = std::max(maxProjectNameWidth, task.project.size());
  msg << "\n";
  msg << "now:" << owner_->TimeToYMDHMS(std::time(0)) << "\n";
  msg << "===============================\n";
  msg << "Waiting tasks:\n";
  msg << std::setw(maxProjectNameWidth) << std::left << "project" << " " << std::setw(15) << std::left << "task_id_last5" << " " << std::setw(20) << std::left << "set_time" << "\n";
  for (const auto &task : wait_tasks_)
  {
    std::string taskIdLast5 = task.id.substr(std::max(static_cast<std::string::size_type>(0), task.id.size() - 5));
    msg << std::setw(maxProjectNameWidth) << std::left << task.project << " " << std::setw(15) << std::left << taskIdLast5 << " " << std::setw(20) << std::left << owner_->TimeToYMDHMS(task.set_time) << "\n";
  }
  msg << "===============================\n";
  msg << "Ready tasks:\n";
  msg << std::setw(maxProjectNameWidth) << std::left << "project" << " " << std::setw(15) << std::left << "task_id_last5" << " " << std::setw(20) << std::left << "set_time" << "\n";
  for (const auto &task : ready_tasks_)
  {
    std::string taskIdLast5 = task.id.substr(std::max(static_cast<std::string::size_type>(0), task.id.size() - 5));
    msg << std::setw(maxProjectNameWidth) << std::left << task.project << " " << std::setw(15) << std::left << taskIdLast5 << " " << std::setw(20) << std::left << owner_->TimeToYMDHMS(task.set_time) << "\n";
  }
  msg << "===============================\n";
  return msg.str();
}

void DeviceManagerTask::CheckReadyTask()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  const auto current_time = std::time(nullptr);
  auto new_end = std::remove_if(wait_tasks_.begin(), wait_tasks_.end(),
                                [&current_time, this](auto &&task)
                                {
                                  if (task.set_time < current_time)
                                  {
                                    this->ready_tasks_.push_back(std::move(task));
                                    return true;
                                  }
                                  return false;
                                });
  wait_tasks_.erase(new_end, wait_tasks_.end());
}
