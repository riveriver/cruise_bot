#!/usr/bin/env python3
import rospy
from cruise_decision.msg import TaskCmd
from std_msgs.msg import Bool

def test_theta_publishers():
    # 初始化ROS节点
    rospy.init_node('test_theta_pub', anonymous=True)
    
    # wakeup_theta_pub = rospy.Publisher('/wakeup_theta', TaskCmd, queue_size=10)
    # rospy.sleep(1)
    # wakeup_cmd = TaskCmd()
    # wakeup_cmd.id = "12345sdag"  # 示例任务ID
    # wakeup_cmd.time = 11234564758293  # 当前时间戳
    # wakeup_cmd.pose = "4F1A"  # 示例路由
    # wakeup_cmd.cmd = "test"  # 示例命令类型
    # rospy.loginfo("Publishing wakeup_theta_pub message")
    # wakeup_theta_pub.publish(wakeup_cmd)

    start_theta_pub = rospy.Publisher('/start_theta', Bool, queue_size=10)
    rospy.sleep(1)

    start_cmd = Bool()
    start_cmd.data = True
    rospy.loginfo("Publishing start_theta_pub message")
    start_theta_pub.publish(start_cmd)
    
    # 保持节点运行，等待消息发布
    rospy.spin()

if __name__ == '__main__':
    try:
        test_theta_publishers()
    except rospy.ROSInterruptException:
        pass