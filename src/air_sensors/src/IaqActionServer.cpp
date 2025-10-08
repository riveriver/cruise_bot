#include <string>
#include <signal.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <ctime>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cruise_msgs/TaskExecuterAction.h>
#include <cruise_msgs/AirSensors.h>

typedef actionlib::SimpleActionServer<cruise_msgs::TaskExecuterAction> TaskExecuterServer;

class IaqActionServer{
public:
    IaqActionServer(const std::string& server_name)
        : server_name_(server_name), nh_(""), 
          server_(nh_, server_name, boost::bind(&IaqActionServer::goalCB, this, _1), false) {
        
        // 设置SIGINT信号处理器
        signal(SIGINT, &IaqActionServer::ctrlCSigintHandler);
        
        // 订阅空气质量数据
        air_sensors_sub_ = nh_.subscribe("/sensors/air_data", 10, &IaqActionServer::airSensorsCallback, this);
        
        // 启动动作服务器
        server_.start();
        ROS_INFO("TaskExecuter %s started.", server_name_.c_str());
        
        // 进入ROS消息处理循环
        ros::spin();
        ROS_ERROR("TaskExecuter %s shutting down.", server_name_.c_str());
    }

private:
    void airSensorsCallback(const cruise_msgs::AirSensors::ConstPtr& msg) {
        latest_air_data_ = *msg;
    }

    static void ctrlCSigintHandler(int sig) {
        ros::shutdown();
    }

    std::string getCurrentTimeString() {
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        return ss.str();
    }

    std::string getResourcePath() {
        std::string command = "rospack find cruise_resources";
        char buffer[128];
        std::string result = "";
        
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            ROS_ERROR("Failed to run rospack command");
            return "/tmp";
        }
        
        while (fgets(buffer, sizeof buffer, pipe) != NULL) {
            result += buffer;
        }
        pclose(pipe);
        
        // 移除换行符
        if (!result.empty() && result.back() == '\n') {
            result.pop_back();
        }
        
        return result + "/data";
    }

    void goalCB(const cruise_msgs::TaskExecuterGoalConstPtr& goal) {
        ROS_INFO("TaskExecuter %s received goal: type=%s, cmd=%s, id=%s", 
                 server_name_.c_str(), goal->type.c_str(), goal->cmd.c_str(), goal->id.c_str());
        
        // 检查是否为空气质量检测任务
        if (goal->type != "air_quality") {
            ROS_WARN("TaskExecuter %s received unsupported task type: %s", 
                     server_name_.c_str(), goal->type.c_str());
            cruise_msgs::TaskExecuterResult result;
            result.code = "1";
            result.message = "Unsupported task type";
            server_.setAborted(result);
            return;
        }

        if (goal->cmd == "start") {
            executeAirQualityCollection(goal);
        } else {
            ROS_WARN("TaskExecuter %s received unsupported command: %s", 
                     server_name_.c_str(), goal->cmd.c_str());
            cruise_msgs::TaskExecuterResult result;
            result.code = "1";
            result.message = "Unsupported command";
            server_.setAborted(result);
        }
    }

    void executeAirQualityCollection(const cruise_msgs::TaskExecuterGoalConstPtr& goal) {
        ROS_INFO("Starting air quality data collection for task: %s", goal->id.c_str());
        
        // 创建数据保存路径
        std::string data_path = getResourcePath();
        std::string timestamp = getCurrentTimeString();
        std::string filename = "air_quality_" + goal->id + "_" + timestamp + ".csv";
        std::string full_path = data_path + "/" + filename;
        
        // 确保目录存在
        std::string mkdir_cmd = "mkdir -p " + data_path;
        system(mkdir_cmd.c_str());
        
        // 打开CSV文件
        std::ofstream csv_file(full_path);
        if (!csv_file.is_open()) {
            ROS_ERROR("Failed to create CSV file: %s", full_path.c_str());
            cruise_msgs::TaskExecuterResult result;
            result.code = "1";
            result.message = "Failed to create CSV file";
            server_.setAborted(result);
            return;
        }
        
        // 写入CSV头部
        csv_file << "timestamp,temperature,humidity,airflow,co2,ozone_concentration,tvoc,pm1_0,pm2_5,pm10,co,no2,formaldehyde,rn,light1,light2,noise\n";
        
        // 数据采集变量
        std::vector<cruise_msgs::AirSensors> collected_data;
        ros::Rate rate(1.0); // 1 Hz
        
        // 采集60秒的数据
        for (int i = 0; i < 60; i++) {
            // 检查是否收到取消请求
            if (server_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("TaskExecuter %s preempted during data collection.", server_name_.c_str());
                csv_file.close();
                
                cruise_msgs::TaskExecuterResult result;
                result.code = "1";
                result.message = "Action preempted";
                server_.setPreempted(result);
                return;
            }
            
            // 收集当前数据
            cruise_msgs::AirSensors current_data = latest_air_data_;
            collected_data.push_back(current_data);
            
            // 写入CSV
            auto now = std::time(nullptr);
            csv_file << now << ","
                     << current_data.temperature << ","
                     << current_data.humidity << ","
                     << current_data.airflow << ","
                     << current_data.co2 << ","
                     << current_data.ozone_concentration << ","
                     << current_data.tvoc << ","
                     << current_data.pm1_0 << ","
                     << current_data.pm2_5 << ","
                     << current_data.pm10 << ","
                     << current_data.co << ","
                     << current_data.no2 << ","
                     << current_data.formaldehyde << ","
                     << current_data.rn << ","
                     << current_data.light1 << ","
                     << current_data.light2 << ","
                     << current_data.noise << "\n";
            
            // 发布进度反馈
            cruise_msgs::TaskExecuterFeedback feedback;
            feedback.progress = (float)(i + 1) / 60.0 * 100.0;
            server_.publishFeedback(feedback);
            
            ROS_INFO("Air quality data collection progress: %.1f%% (%d/60)", feedback.progress, i + 1);
            
            rate.sleep();
        }
        
        csv_file.close();
        
        ROS_INFO("Air quality data collection completed. Data saved to: %s", full_path.c_str());
        
        // 设置成功结果
        cruise_msgs::TaskExecuterResult result;
        result.code = "0";
        result.message = "Air quality data collection completed successfully. File: " + filename;
        server_.setSucceeded(result);
    }

    ros::NodeHandle nh_;
    TaskExecuterServer server_;
    std::string server_name_;
    ros::Subscriber air_sensors_sub_;
    cruise_msgs::AirSensors latest_air_data_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "iaq_action_server_node");
    IaqActionServer iaq_action_server("task_executer/air_quality");
    return 0;
}