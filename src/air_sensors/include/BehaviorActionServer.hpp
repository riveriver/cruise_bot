#include <string>
#include <signal.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cruise_msgs/BehaviorExecuterAction.h>
#include <cruise_msgs/AirSensors.h>

typedef actionlib::SimpleActionServer<cruise_msgs::BehaviorExecuterAction> BehaviorExecuter;

class BehaviorActionServer{
public:
    BehaviorActionServer(const std::string& server_name)
        : server_name_(server_name),nh_(""), server_(nh_,server_name, boost::bind(&BehaviorActionServer::goalCB, this, _1), false) {
        // 设置SIGINT信号处理器
        signal(SIGINT, &BehaviorActionServer::ctrlCSigintHandler);
        air_sensors_sub_ = nh_.subscribe("/sensors/air_data", 10, &BehaviorActionServer::airSensorsCallback, this);
        air_result_pub_ = nh_.advertise<cruise_msgs::AirSensors>("/sensors/air_result", 3);
        // 启动动作服务器
        server_.start();
        ROS_INFO("BehaviorExecuter %s started.",server_name_.c_str());
        // 进入ROS消息处理循环，保证动作服务器可以接收消息
        ros::spin();
        ROS_ERROR("BehaviorExecuter %s shutting down.",server_name_.c_str());
    }

private:
    void airSensorsCallback(const cruise_msgs::AirSensors::ConstPtr& msg) {
        latest_air_data = *msg;
    }

    static void ctrlCSigintHandler(int sig) {
        // 自定义动作，例如发布停止消息给其他节点
        // 调用ROS的shutdown函数
        ros::shutdown();
    }

    void goalCB(const cruise_msgs::BehaviorExecuterGoalConstPtr& goal) {
        ROS_INFO("BehaviorExecuter %s received a goal: %s", server_name_.c_str(), goal->task_project.c_str());
        
        avg = latest_air_data;
        // 创建循环来模拟执行过程
        for (float progress = 0.0; progress <= 100.0; progress += 10.0) {
            
            // 更新平均空气数据
            cruise_msgs::AirSensors latest = latest_air_data;
            avg.temperature = (avg.temperature + latest.temperature)/2;
            avg.humidity = (avg.humidity + latest.humidity)/2;
            avg.co2 = (avg.co2 + latest.co2)/2;
            avg.tvoc = (avg.tvoc + latest.tvoc)/2;
            avg.formaldehyde = (avg.formaldehyde + latest.formaldehyde)/2;

            // 发布反馈消息
            cruise_msgs::BehaviorExecuterFeedback feedback;
            feedback.progress = progress;
            server_.publishFeedback(feedback);

            // 检查是否收到取消请求
            if (server_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("BehaviorExecuter %s preempted.", server_name_.c_str());

                // 设置服务器为被抢占状态
                cruise_msgs::BehaviorExecuterResult result;
                result.progress = progress;
                result.code = 1;  // 自定义错误代码，表示被抢占
                result.msg = "Action preempted";
                server_.setPreempted(result);
                return;
            }

            // 模拟执行时间
            ros::Duration(1.0).sleep();
        }

        // 如果执行完成，设置结果
        air_result_pub_.publish(avg);
        cruise_msgs::BehaviorExecuterResult result;
        result.progress = 100.0;
        result.code = 0;  // 表示成功
        result.msg = "Success";
        server_.setSucceeded(result);
    }

    ros::NodeHandle nh_;
    BehaviorExecuter server_;
    std::string server_name_;
    ros::Subscriber air_sensors_sub_;
    ros::Publisher air_result_pub_;
    cruise_msgs::AirSensors latest_air_data;
    cruise_msgs::AirSensors avg;
};

