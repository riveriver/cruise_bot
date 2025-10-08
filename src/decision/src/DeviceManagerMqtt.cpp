/*
 * DeviceManagerMqtt.cpp
 * MQTT-related implementations for DeviceManager moved here for modularity.
 */

#include "DeviceManager.hpp"
#include "DeviceManagerTask.hpp"
#include "DeviceManagerMqtt.hpp"
#include <jsoncpp/json/json.h>
#include <mosquitto.h>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <functional>

// DeviceManagerMqtt 实现
DeviceManagerMqtt::DeviceManagerMqtt() : mosq_(nullptr) {}

DeviceManagerMqtt::~DeviceManagerMqtt()
{
  destroy();
}

int8_t DeviceManagerMqtt::init(const std::string &host, int port)
{
  mosquitto_lib_init();

  mosq_ = mosquitto_new(NULL, true, this);
  if (!mosq_)
  {
    std::cerr << "create client failed.." << std::endl;
    mosquitto_lib_cleanup();
    return 0x01;
  }

  mosquitto_message_callback_set(mosq_, DeviceManagerMqtt::MessageCallback);
#ifdef MQTT_DEBUG
  mosquitto_log_callback_set(mosq_, DeviceManagerMqtt::LogCallback);
#endif

  if (mosquitto_connect(mosq_, host.c_str(), port, 60))
  {
    std::cerr << "Unable to connect." << std::endl;
    return 0x02;
  }

  int loop = mosquitto_loop_start(mosq_);
  if (loop != MOSQ_ERR_SUCCESS)
  {
    std::cerr << "mosquitto loop error" << std::endl;
    return 0x03;
  }

  return 0;
}

void DeviceManagerMqtt::destroy()
{
  if (mosq_)
  {
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
  }
  mosquitto_lib_cleanup();
}

void DeviceManagerMqtt::subscribe(const std::string &topic, std::function<void(const struct mosquitto_message *)> cb, int qos)
{
  if (!mosq_)
    return;
  mosquitto_subscribe(mosq_, NULL, topic.c_str(), qos);
  topicCallbacks_[topic] = cb;
}

bool DeviceManagerMqtt::publish(const std::string &topic, const std::string &payload, int qos)
{
  if (!mosq_)
    return false;

  // Validate inputs to avoid MOSQ_ERR_INVAL (Invalid function arguments)
  if (topic.empty())
  {
    ROS_ERROR("Mosquitto publish called with empty topic");
    return false;
  }

  if (qos < 0 || qos > 2)
  {
    ROS_ERROR("Mosquitto publish called with invalid qos=%d, must be 0..2", qos);
    return false;
  }

  int payload_len = static_cast<int>(payload.size());
  const void *payload_ptr = payload_len > 0 ? static_cast<const void *>(payload.c_str()) : NULL;
  int ret = mosquitto_publish(mosq_, NULL, topic.c_str(), payload_len, payload_ptr, qos, 0);
  if (ret != MOSQ_ERR_SUCCESS)
  {
    // 打印更详细的错误信息，便于诊断
    const char *errstr = mosquitto_strerror(ret);
    ROS_ERROR("Mosquitto publish failed to '%s' (rc=%d): %s", topic.c_str(), ret, errstr ? errstr : "");
    // 若未连接，尝试异步重连
    if (ret == MOSQ_ERR_NO_CONN)
    {
      ROS_WARN("Mosquitto not connected, attempting reconnect...");
      int r = mosquitto_reconnect_async(mosq_);
      if (r != MOSQ_ERR_SUCCESS)
      {
        ROS_ERROR("mosquitto_reconnect_async failed: %s", mosquitto_strerror(r));
      }
    }
    return false;
  }
  return true;
}

void DeviceManagerMqtt::LogCallback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
  std::cout << str << std::endl;
}

void DeviceManagerMqtt::MessageCallback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg)
{
  DeviceManagerMqtt *self = static_cast<DeviceManagerMqtt *>(userdata);
  if (!self)
    return;

  std::string topic(msg->topic);
  char *data = (char *)msg->payload;

  if (topic != "device/prototypes001/info")
    std::cout << "[sub] " << topic << ": " << data << std::endl;

  auto it = self->topicCallbacks_.find(topic);
  if (it != self->topicCallbacks_.end())
  {
    it->second(msg);
  }
  else
  {
    if (msg->payloadlen)
    {
      std::cerr << "Received msg on topic '" << topic << "': " << data << std::endl;
    }
  }
}

void DeviceManager::UploadAirData()
{
  MqttMsgHead msg_head;
  msg_head.id = "0";
  msg_head.topic = "response/prototypes001/sensor";
  msg_head.qos = 0;
  msg_head.type = "sensor";
  msg_head.code = 0;
  Json::Value payload;
  PackAirData(iaq_data, payload);
  PubAckMsg(msg_head, payload);
}

void DeviceManager::UploadAirCallback(const ros::TimerEvent &)
{
  UploadAirData();
}

void DeviceManager::mqtt2rosCmdCallback(const struct mosquitto_message *msg)
{
  if (!msg->payloadlen)
    return;

  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  char *data_char = (char *)msg->payload;
  std::istringstream data_stream(data_char);
  bool success = Json::parseFromStream(reader, data_stream, &root, &errs);
  if (!success)
  {
    std::cerr << "Failed to parse JSON: " << errs << std::endl;
    return;
  }

  MqttMsgHead msg_head;
  Json::Value payload;
  msg_head.topic = "response/prototypes001/command";
  msg_head.qos = 1;
  msg_head.type = root["type"].asString();
  msg_head.id = root["messageId"].asString();

  static std::string expect_type = "cmd";
  if (msg_head.type != expect_type)
  {
    msg_head.code = 0x11;
    msg_head.msg = msg_head.id + ":" + msg_head.type + "/" + expect_type;
    PubAckMsg(msg_head, payload);
    ROS_ERROR("%s", msg_head.msg.c_str());
    return;
  }

  std::string data_type = root["data"]["type"].asString();
  payload["type"] = data_type;
  std::string action_type = root["data"]["actionType"].asString();
  payload["actionType"] = action_type;

  if (data_type == "liveStream" && action_type == "apply")
    SetLiveStream(true);
  else if (data_type == "liveStream" && action_type == "over")
    SetLiveStream(false);
  else if (data_type == "displayMode" && action_type == "apply")
    SetAirUpload(true);
  else if (data_type == "displayMode" && action_type == "over")
    SetAirUpload(false);
  else if (data_type == "control" && action_type == "apply")
    SetControlMode(true);
  else if (data_type == "control" && action_type == "over")
    SetControlMode(false);
  else if (data_type == "HomeMode")
    SetHomeMode(true);
  else if (data_type == "StopMode")
    SetStopMode(true);
  else
  {
    msg_head.code = 0x21;
    msg_head.msg = "[ERROR]" + data_type + ":" + action_type;
    PubAckMsg(msg_head, payload);
    ROS_ERROR("%s", msg_head.msg.c_str());
    return;
  }
  msg_head.code = 0;
  msg_head.msg = msg_head.id;
  PubAckMsg(msg_head, payload);
}

void DeviceManager::mqtt2rosInfoCallback(const struct mosquitto_message *msg)
{
  if (!msg->payloadlen)
    return;

  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  char *data_char = (char *)msg->payload;
  std::istringstream data_stream(data_char);
  bool success = Json::parseFromStream(reader, data_stream, &root, &errs);
  if (!success)
  {
    std::cerr << "Failed to parse JSON: " << errs << std::endl;
    return;
  }

  MqttMsgHead msg_head;
  Json::Value payload;
  static std::string expect_type = "info";
  msg_head.topic = "response/prototypes001/info";
  msg_head.type = root["type"].asString();
  msg_head.id = root["messageId"].asString();

  if (msg_head.type != expect_type)
  {
    msg_head.code = 1;
    msg_head.msg = msg_head.id + "" + msg_head.type + "/" + expect_type;
    payload = "";
    PubAckMsg(msg_head, payload);
    ROS_ERROR("%s", msg_head.msg.c_str());
    return;
  }
  msg_head.code = 0;
  msg_head.msg = msg_head.id;
  PackDeviceInfo(payload);
  PubAckMsg(msg_head, payload);
}

void DeviceManager::mqtt2rosMoveCallback(const struct mosquitto_message *msg)
{
  if (!msg->payloadlen)
    return;

  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  char *data_char = (char *)msg->payload;
  std::istringstream data_stream(data_char);
  bool success = Json::parseFromStream(reader, data_stream, &root, &errs);
  if (!success)
  {
    std::cerr << "Failed to parse JSON: " << errs << std::endl;
    return;
  }

  MqttMsgHead msg_head;
  Json::Value payload;
  static std::string expect_type = "move";
  msg_head.topic = "response/prototypes001/move";
  msg_head.type = root["type"].asString();
  msg_head.id = root["messageId"].asString();

  if (msg_head.type != expect_type)
  {
    msg_head.code = 1;
    msg_head.msg = msg_head.id + " Received message is not expect type:" + msg_head.type + "/" + expect_type;
    payload = "";
    PubAckMsg(msg_head, payload);
    ROS_ERROR("%s", msg_head.msg.c_str());
    return;
  }

  Json::Value data = root["data"];
  cmd_move.speed = data["speed"].asInt();
  cmd_move.stop = data["cmd"][0].asBool();
  cmd_move.up = data["cmd"][1].asBool();
  cmd_move.down = data["cmd"][2].asBool();
  cmd_move.left = data["cmd"][3].asBool();
  cmd_move.right = data["cmd"][4].asBool();
  cmd_move.rotate_left = data["cmd"][5].asBool();
  cmd_move.rotate_right = data["cmd"][6].asBool();
  cmd_move.rev_time = std::time(0);
}

void DeviceManager::mqtt2rosTaskCallback(const struct mosquitto_message *msg)
{
  if (!msg->payloadlen)
    return;

  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  char *data_char = (char *)msg->payload;
  std::istringstream data_stream(data_char);
  if (!Json::parseFromStream(reader, data_stream, &root, &errs))
  {
    ROS_ERROR("Failed to parse JSON");
    return;
  }
  Json::Value data = root["data"];
  std::string cmd = data["cmd"].asString();
  std::string taskId = data["taskId"].asString();
  Json::LargestInt setTime = data["setTime"].asLargestInt();

  MqttMsgHead msg_head;
  Json::Value payload;
  msg_head.topic = "response/prototypes001/task";
  msg_head.type = root["type"].asString();
  msg_head.id = root["messageId"].asString();
  payload["taskId"] = taskId;

  if (cmd == "add")
  {
    if (task_helper_)
    {
      for (const auto &t : task_helper_->ListWaitTasks())
      {
        if (t.id == taskId)
        {
          msg_head.code = 1;
          msg_head.msg = "Task ID already exists in the queue.";
          payload["state"] = "reject";
          PubAckMsg(msg_head, payload);
          break;
        }
      }
    }
    TaskInfo task;
    task.id = taskId;
    task.set_time = setTime;

    const Json::Value &projects = data["project"];
    for (Json::Value::const_iterator itProject = projects.begin();
         itProject != projects.end(); ++itProject)
    {
      const std::string &project = itProject->asString();
      task.actions.push_back(project);
    }

    if (data["poseMode"] == "spot")
    {
      task.pose_mode = "spot";
      const Json::Value &pose_array = data["spot"];
      for (Json::Value::const_iterator itArea = pose_array.begin();
           itArea != pose_array.end(); ++itArea)
      {
        const Json::Value &p = *itArea;
        SpotPoseDefine s;
        s.floor = p[0].asInt();
        s.x = p[1].asInt();
        s.y = p[2].asInt();
        task.spots.push_back(s);
      }
    }
    else
    {
      task.pose_mode = "area";
      const Json::Value &pose_array = data["area"];
      for (Json::Value::const_iterator itArea = pose_array.begin();
           itArea != pose_array.end(); ++itArea)
      {
        const Json::Value &p = *itArea;
        AreaPoseDefine a;
        a.floor = p[0].asInt();
        a.num = p[1].asInt();
        task.areas.push_back(a);
      }
    }
  if (task_helper_) task_helper_->AddTasKInQueue(task);
    ROS_INFO("Task actions:");
    for (const auto& action : task.actions) {
        ROS_INFO(" - %s", action.c_str());
    }
    ROS_INFO("Task areas:");
    for (const auto& area : task.areas) {
        ROS_INFO(" - Floor: %d, Num: %d", area.floor, area.num);
    }
    payload["state"] = "accept";
    PubAckMsg(msg_head, payload);
    return;
  }

  if (cmd == "front")
  {
    TaskInfo task;
    task.id = data["taskId"].asString();
  if (task_helper_) task_helper_->MoveTaskFrontQueue(task);
    payload["state"] = "front";
    PubAckMsg(msg_head, payload);
    return;
  }

  if (cmd == "remove")
  {
    TaskInfo task;
    task.id = data["taskId"].asString();
  if (task_helper_) task_helper_->RemoveTaskInQueue(task);
    payload["state"] = "remove";
    PubAckMsg(msg_head, payload);
    return;
  }

  msg_head.code = 1;
  msg_head.msg = "error cmd:" + cmd;
  PubAckMsg(msg_head, payload);
  return;
}

void DeviceManager::mqtt2rosSensorCallback(const struct mosquitto_message *msg)
{
  if (!msg->payloadlen)
    return;

  Json::CharReaderBuilder reader;
  Json::Value root;
  std::string errs;
  char *data_char = (char *)msg->payload;
  std::istringstream data_stream(data_char);
  bool success = Json::parseFromStream(reader, data_stream, &root, &errs);
  if (!success)
  {
    std::cerr << "Failed to parse JSON: " << errs << std::endl;
    return;
  }

  MqttMsgHead msg_head;
  Json::Value payload;
  static std::string expect_type = "sensor";
  msg_head.topic = "response/prototypes001/sensor";
  msg_head.type = root["type"].asString();
  msg_head.id = root["messageId"].asString();

  if (msg_head.type != expect_type)
  {
    msg_head.code = 1;
    msg_head.msg = msg_head.id + " Received message is not expect type:" + msg_head.type + "/" + expect_type;
    payload = "";
    PubAckMsg(msg_head, payload);
    ROS_ERROR("%s", msg_head.msg.c_str());
    return;
  }

  msg_head.code = 0;
  msg_head.msg = msg_head.id;

  PackAirData(iaq_data, payload);
  PubAckMsg(msg_head, payload);
}
