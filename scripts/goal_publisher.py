#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('robot_arm_goal', Pose, queue_size=10)
    rospy.init_node('goal_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Pose()
    msg.position.x = -0.391
    msg.position.y = -0.051
    msg.position.z = 0.652
    msg.orientation.x = 0.
    msg.orientation.y = 0.
    msg.orientation.z = 0.
    msg.orientation.w = 0.899
    for i in range(5):
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass