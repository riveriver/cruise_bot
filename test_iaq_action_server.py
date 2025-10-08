#!/usr/bin/env python3

import rospy
import actionlib
from cruise_msgs.msg import TaskExecuterAction, TaskExecuterGoal

def test_air_quality_action():
    rospy.init_node('test_iaq_action_client')
    
    # 创建 action client
    client = actionlib.SimpleActionClient('task_executer/air_quality', TaskExecuterAction)
    
    # 等待服务器启动
    rospy.loginfo("Waiting for air_quality action server...")
    client.wait_for_server()
    rospy.loginfo("Air quality action server found!")
    
    # 创建目标
    goal = TaskExecuterGoal()
    goal.type = "air_quality"
    goal.cmd = "start"
    goal.id = "test_task_001"
    goal.pose = "0.0,0.0,0.0"
    goal.time = "00:00:00"
    
    rospy.loginfo("Sending goal: %s", goal)
    
    # 发送目标
    client.send_goal(goal)
    
    # 等待结果
    rospy.loginfo("Waiting for result...")
    result = client.wait_for_result(timeout=rospy.Duration(70))  # 等待70秒
    
    if result:
        final_result = client.get_result()
        rospy.loginfo("Action completed with result: code=%s, message=%s", 
                     final_result.code, final_result.message)
    else:
        rospy.logwarn("Action did not complete within timeout")
        client.cancel_goal()

if __name__ == '__main__':
    try:
        test_air_quality_action()
    except rospy.ROSInterruptException:
        pass
