#pragma once

#include <functional>
#include <unordered_map>
#include <string>
#include <mosquitto.h>

// 一个轻量的 MQTT 客户端封装类，支持订阅回调与发布
class DeviceManagerMqtt
{
public:
	DeviceManagerMqtt();
	~DeviceManagerMqtt();

	// 初始化连接到 broker，返回 0 表示成功，非 0 表示错误码
	int8_t init(const std::string &host = "129.226.138.87", int port = 1883);

	// 清理资源
	void destroy();

	// 订阅一个 topic，并注册回调
	void subscribe(const std::string &topic, std::function<void(const struct mosquitto_message *)> cb, int qos = 2);

	// 发布任意字符串消息
	bool publish(const std::string &topic, const std::string &payload, int qos = 0);

private:
	struct mosquitto *mosq_ = nullptr;
	std::unordered_map<std::string, std::function<void(const struct mosquitto_message *)>> topicCallbacks_;

	static void LogCallback(struct mosquitto *mosq, void *userdata, int level, const char *str);
	static void MessageCallback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg);
};
