#!/usr/bin/env python3

from controller import RobotController
import rospy

if __name__ == "__main__":
    rospy.init_node('track_marker')
    controller = RobotController()
    controller.get_goal_pose_array('/camera/goal_pose_array')
    result = controller.goto_pose_array()
    rospy.loginfo(result)
