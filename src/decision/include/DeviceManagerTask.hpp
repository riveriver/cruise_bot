#pragma once
#include <deque>
#include "DeviceManager.hpp"

class DeviceManagerTask
{
public:
  explicit DeviceManagerTask(DeviceManager *owner);
  bool PubTask();
  bool PubRoute();
  uint8_t AddTasKInQueue(const TaskInfo &obj);
  uint8_t MoveTaskFrontQueue(const TaskInfo &obj);
  uint8_t RemoveTaskInQueue(const TaskInfo &obj);
  TaskInfo PeekFrontReadyTasks();
  TaskInfo PopFrontReadyTasks();

  // convenience accessors for inspection
  std::deque<TaskInfo> ListWaitTasks();
  std::deque<TaskInfo> ListReadyTasks();
  std::string GetTaskInfoSimple();
  std::string GetTaskInfo();
  void CheckReadyTask();

private:
  DeviceManager *owner_;
  // own task storage and synchronization
  std::deque<TaskInfo> wait_tasks_;
  std::deque<TaskInfo> ready_tasks_;
  std::mutex queue_mutex_;
};
