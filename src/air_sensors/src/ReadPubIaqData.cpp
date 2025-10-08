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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cruise_msgs/AirSensors.h"

#include "modbus.h"

struct ModbusSensorInfo {
    std::string type; 
    std::string port_name;
    int slave_id;
    modbus_t* handle;
    bool initialized;
};

cruise_msgs::AirSensors air_data;

void read_sensor_temp(modbus_t* interface){
    uint16_t data[2];
    if(modbus_read_registers(interface,0x0001,0x0001,data)!=(-1)){
        air_data.temperature = (data[0] - 2000) / 100.0;
    }else{
      ROS_ERROR("Failed to read temperature");
    }

    if(modbus_read_registers(interface,0x0002,0x0001,data)!=(-1)){
        air_data.humidity = data[0]  / 100.0;
    }else{
      ROS_ERROR("Failed to read humidity");
    }


    if(modbus_read_registers(interface,0x0012,0x0001,data)!=(-1)){
        ROS_INFO("[raw]CO: %X %X", data[0], data[1]);
        air_data.co= data[0];
    }else{
      ROS_ERROR("Failed to read co");
    }

    if(modbus_read_registers(interface,0x0013,0x0001,data)!=(-1)){
        air_data.co2= data[0];
    }else{
      ROS_ERROR("Failed to read co2");
    }

    if(modbus_read_registers(interface,0x0016,0x0001,data)!=(-1)){
        air_data.formaldehyde = data[0] / 100.0;
    }else{
      ROS_ERROR("Failed to read formaldehyde");
    }

    if(modbus_read_registers(interface,0x000E,0x0001,data)!=(-1)){
        air_data.tvoc = data[0] / 100.0;
    }else{
      ROS_ERROR("Failed to read tvoc");
    }

    if(modbus_read_registers(interface,0x0007,0x0001,data)!=(-1)){
        air_data.pm1_0 = data[0];
    }else{
      ROS_ERROR("Failed to read pm1.0");
    }

    if(modbus_read_registers(interface,0x0008,0x0001,data)!=(-1)){
        air_data.pm2_5 = data[0];
    }else{
      ROS_ERROR("Failed to read pm2.5");
    }

    if(modbus_read_registers(interface,0x0009,0x0001,data)!=(-1)){
        air_data.pm10 = data[0];
    }else{
      ROS_ERROR("Failed to read pm10");
    }
}

void read_sensor_lux_all(modbus_t* interface){
    uint16_t data[2];
    if(modbus_read_registers(interface,0x0003,0x0002,data)!=(-1)){
        air_data.light1 =  (data[1] == 0 && data[0] < 65536) ? data[0] : (data[1] * 65536 + data[0]);
    }else{
      // ROS_ERROR("Failed to read Lux Sensor 0x0003");
    }

    if(modbus_read_registers(interface,0x0006,0x0001,data)!=(-1)){
        air_data.noise = data[0] /10.0;
    }else{
      // ROS_ERROR("Failed to read Lux Sensor 0x0006");
    }

    if(modbus_read_registers(interface,0x0017,0x0001,data)!=(-1)){
        air_data.ozone_concentration = data[0] / 100.0;
    }else{
      // ROS_ERROR("Failed to read Lux Sensor 0x0017");
    }
}

void read_sensor_lux_only(modbus_t* interface){
    uint16_t data[2];
    if(modbus_read_registers(interface,0x0003,0x0002,data)!=(-1)){
        air_data.light2 =  (data[1] == 0 && data[0] < 65536) ? data[0] : (data[1] * 65536 + data[0]);
    }else{
      // ROS_ERROR("Failed to read Lux Sensor 0x0003");
    }
}


void read_sensor_no2(modbus_t* interface){
    uint16_t data[2];
    if(modbus_read_registers(interface,0x0000,0x0002,data)!=(-1)){
       ROS_INFO("[raw]NO2: %X %X", data[0], data[1]);
        air_data.no2 =  (data[1] * 65536 + data[0])/10;
    }else{
      ROS_ERROR("Failed to read no2");
    }
}

void read_sensor_rn(modbus_t* interface){
    uint16_t data[2];
    if(modbus_read_registers(interface,0x0006,0x0001,data)!=(-1)){
        ROS_INFO("[raw]RN: %X %X", data[0], data[1]);
        air_data.rn =  (data[1] * 65536 + data[0]);
    }else{
      ROS_ERROR("Failed to read rn"); 
    }
}

void read_sensor_aws(modbus_t* interface){
    uint16_t data[2];
    if(modbus_read_registers(interface,0x0063,0x0001,data)!=(-1)){
        air_data.airflow =  (data[1] * 65536 + data[0])  / 1000.0;
    }else{
      // ROS_ERROR("Failed to read aws");
    }
}

int main(int argc, char** argv) {
  /* ros init */
  ros::init(argc, argv, "PubIaqData");
  ros::NodeHandle nh;
  ros::Publisher iaq_pub =
    nh.advertise<cruise_msgs::AirSensors>("/sensors/air_data", 3);

  /* modbus init */
  /* Define the Modbus sensor initialization information */
    std::vector<ModbusSensorInfo> sensors_table = {
        {"Rn_Sensor", "/dev/ttyACM0", 1, NULL, false},
        {"LuxAll_Sensor", "/dev/ttyACM3", 2, NULL, false},
        {"Temp_Sensor","/dev/ttyACM5", 3, NULL, false},
        {"Aws_Sensor" , "/dev/ttyACM7", 1, NULL, false},
        // {"LuxOnly_Sensor", "/dev/ttyACM5", 2, NULL, false},
        {"No2_Sensor", "/dev/ttyACM2", 3, NULL, false},
    };

  /* Modbus init using a for loop */
  for (auto& sensor : sensors_table) {
    sensor.handle = modbus_new_rtu(sensor.port_name.c_str(), 9600, 'N', 8, 1);
    if (sensor.handle == NULL) {
      ROS_ERROR_STREAM(
        "Failed to create Modbus context for port: " << sensor.port_name);
      continue;
    }

    if (modbus_set_slave(sensor.handle, sensor.slave_id) == -1) {
      ROS_ERROR_STREAM("Failed to set slave ID for port: " << sensor.port_name);
      modbus_free(sensor.handle);
      sensor.handle = NULL;
      continue;
    }

    if (modbus_connect(sensor.handle) == -1) {
      ROS_ERROR_STREAM(
        "Failed to connect to Modbus device on port: " << sensor.port_name);
      modbus_free(sensor.handle);
      sensor.handle = NULL;
      continue;
    }
    
    uint16_t check_read[2];
    if(modbus_read_registers(sensor.handle,0X0001,0X0001,check_read)!=(-1)){
				// TODO: 读通用地址，校验是否能读取
    } 

    sensor.initialized = true;
    ROS_INFO_STREAM(
      "Modbus sensor initialized successfully on port: " << sensor.port_name);
  }

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    for (auto& sensor : sensors_table) {
        if (sensor.initialized) {
            if (sensor.type == "No2_Sensor") {
                read_sensor_no2(sensor.handle);
            } else if (sensor.type == "LuxAll_Sensor") {
                read_sensor_lux_all(sensor.handle);
            } else if (sensor.type == "Temp_Sensor") {
                read_sensor_temp(sensor.handle);
            } else if (sensor.type == "Rn_Sensor") {
                read_sensor_rn(sensor.handle);
            }
            else if (sensor.type == "Aws_Sensor") {
                read_sensor_aws(sensor.handle);
            }
            else if (sensor.type == "LuxOnly_Sensor") {
                read_sensor_lux_only(sensor.handle);
            }
            else {
                ROS_ERROR("Unknown sensor type: %s", sensor.type.c_str());
            }
        }
    }

    // std::lock_guard<std::mutex> lock(save_iaq_file_mutex);
    // std::stringstream ss;
    // ss << std::time(0) << ","
    // << air_data.temperature << ","
    // << air_data.humidity << ","
    // << air_data.tvoc << ","
    // << air_data.co2 << ","
    // << air_data.formaldehyde << ","
    // << air_data.light1 << ","
    // << air_data.light2 << ","
    // << air_data.noise << ","
    // << air_data.ozone_concentration << "\n";
    // save_iaq_file << ss.str();

    air_data.header.stamp = ros::Time::now();
    iaq_pub.publish(air_data);
  }
  /* Cleanup */
  for (auto& sensor : sensors_table) {
    if (sensor.initialized) {
      modbus_close(sensor.handle);
      modbus_free(sensor.handle);
    }
  }

  return 0;
}
