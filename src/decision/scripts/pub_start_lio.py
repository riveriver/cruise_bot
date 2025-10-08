#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def pub_start_lio():
    # 初始化ROS节点
    rospy.init_node('pub_start_lio', anonymous=True)
    
    # 创建发布者
    start_lio_pub = rospy.Publisher('/start_lio', Bool, queue_size=1)
    
    # 等待发布者准备就绪
    rospy.sleep(1)

    # 测试 start_lio_pub
    start_cmd = Bool()
    start_cmd.data = True
    rospy.loginfo("Publishing start_lio True")
    start_lio_pub.publish(start_cmd)
    
    rospy.sleep(3)  # 给足够时间让消息发布
    rospy.loginfo("Publish completed, exiting node")

if __name__ == '__main__':
    try:
        pub_start_lio()
    except rospy.ROSInterruptException:
        pass