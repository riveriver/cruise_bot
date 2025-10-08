#include <time.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>    // 用于线程安全
#include <sstream>  // 用于字符串流
#include <vector>
#include <random>  // 添加随机数生成器

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cruise_msgs/AirSensors.h"

// 生成指定范围内的随机浮点数（基于原值的10%波动）
double generateRandomValue(double base) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    double range = base * 0.1;  // 10%的波动范围
    std::uniform_real_distribution<> dis(-range, range);
    return base + dis(gen);
}

// 生成指定范围内的随机整数（基于原值的10%波动）
int generateRandomIntValue(int base) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    int range = std::max(1, static_cast<int>(base * 0.1));  // 确保至少有1的波动
    std::uniform_int_distribution<> dis(-range, range);
    return base + dis(gen);
}

int main(int argc, char** argv) {
  /* ros init */
  ros::init(argc, argv, "FakePubIaqData");
  ros::NodeHandle nh;
  ros::Publisher iaq_pub =
    nh.advertise<cruise_msgs::AirSensors>("/sensors/air_data", 10);
  ros::Rate loop_rate(1);
  ROS_INFO("Starting FakePubIaqData node...");
  while (ros::ok()) {
    cruise_msgs::AirSensors air_data;
    air_data.airflow = generateRandomValue(0.3);              // 在0.27-0.33 m/s之间
    air_data.co2 = generateRandomIntValue(600);               // 在540-660 ppm之间
    air_data.co = generateRandomValue(3);                     // 在2.7-3.3 mg/m³之间
    air_data.formaldehyde = generateRandomValue(0.05);        // 在0.045-0.055 mg/m³之间
    air_data.tvoc = generateRandomValue(0.6);                 // 在0.54-0.66 mg/m³之间
    air_data.pm1_0 = generateRandomIntValue(20);              // 在18-22 μg/m³之间
    air_data.pm2_5 = generateRandomIntValue(35);              // 在31-39 μg/m³之间
    air_data.pm10 = generateRandomIntValue(70);               // 在63-77 μg/m³之间
    air_data.temperature = generateRandomValue(22);           // 在19.8-24.2 °C之间
    air_data.humidity = generateRandomIntValue(60);           // 在54-66%之间
    air_data.light1 = generateRandomIntValue(200);            // 在180-220 lux之间
    air_data.noise = generateRandomIntValue(65);              // 在58-72 dB之间
    air_data.ozone_concentration = generateRandomValue(0.06); // 在0.054-0.066 mg/m³之间
    air_data.no2 = generateRandomValue(0.08);                 // 在0.072-0.088 mg/m³之间
    air_data.rn = generateRandomIntValue(40);                 // 在36-44 Bq/m³之间
    air_data.light2 = generateRandomIntValue(200);            // 在180-220 lux之间
    air_data.header.stamp = ros::Time::now();
    iaq_pub.publish(air_data);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}