#! /usr/bin/env python
from __future__ import print_function

import moveit_commander
import geometry_msgs.msg

import numpy as np




try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL

class AcquisitionControl(object):
    """AcquisitionControl"""

    def __init__(self):
        super(AcquisitionControl
, self).__init__()

        robot = moveit_commander.RobotCommander()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing end factor pose")
        print(move_group.get_current_pose().pose)
        print("")

        # Misc variables
        self.box_name = "tablet"
        self.robot = robot
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

if __name__ == '__main__':
    import rospy
    rospy.init_node('get_pose')
    controller = AcquisitionControl()
    while True:
        print(controller.move_group.get_current_pose().pose)