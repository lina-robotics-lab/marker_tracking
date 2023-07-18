#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

# The home position joint states
# - elbow_joint
# - shoulder_lift_joint
# - shoulder_pan_joint
# - wrist_1_joint
# - wrist_2_joint
# - wrist_3_joint

# -3.480052328086458e-05, 
# -1.570824762383932, 
# 3.617435504565947e-05, 
# -1.5708095035948695, 
# -4.369417299443512e-05, 
# -6.500874654591371e-06

# The home eff position 
# position: 
#   x: -5.473312144697357e-05
#   y: 0.2328999993109313
#   z: 1.0793999980600475
# orientation: 
#   x: -0.7071067784851188
#   y: -4.5797273069955207e-05
#   z: -5.713960074521291e-05
#   w: 0.7071067800962404



def talker():
    pub = rospy.Publisher('robot_arm_goal', Pose, queue_size=10)
    rospy.init_node('goal_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Pose()
    msg.position.x = 0.
    msg.position.y = 0.2329
    msg.position.z = 0.950
    msg.orientation.x = -0.707
    msg.orientation.y = 0.
    msg.orientation.z = 0.
    msg.orientation.w = 0.707
    for i in range(5):
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass