#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from eye_tracking_server.msg import GoToPoseAction, GoToPoseGoal
import actionlib




def start_listen_goal_and_publish():
    
    rospy.init_node('GoalLister')
    client = actionlib.SimpleActionClient('GoTo', GoToPoseAction)
    client.wait_for_server()
    print("Server ready")
    while True:
        goal_pose = rospy.wait_for_message('robot_arm_goal', Pose, timeout=30.)
        print("I heared {}".format(goal_pose))
        goal = GoToPoseGoal()
        goal.position_x = goal_pose.position.x
        goal.position_y = goal_pose.position.y
        goal.position_z = goal_pose.position.z
        goal.orientation_x = goal_pose.orientation.x
        goal.orientation_y = goal_pose.orientation.y
        goal.orientation_z = goal_pose.orientation.z
        goal.orientation_w = goal_pose.orientation.w
        client.send_goal(goal)
        client.wait_for_result()
        print(client.get_result)

if __name__ == '__main__':
    start_listen_goal_and_publish()
