#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pickle as pkl
import numpy as np

from region import AcquisitionRegion
from controller import AcquisitionControl

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


def main():
    N_corners = 8
    corners = []
    corner_joint_values = []
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the Eye Tracker Data Acquision Interface")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to start the moveit_commander ..."
        )
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("robotControl", anonymous=True)

        tutorial = AcquisitionControl()

        load_corners = input(
            "============ Use previously stored corners?(Y/N)"
        )
        if load_corners in['Y','y']:
            fn = input('Please input the path to the .pkl file storing the corners:')
            with open(fn,'rb') as f:
                corners = pkl.load(f)
            print('Corner coordinates are loaded from {}'.format(fn))
        else:
            for i in range(N_corners):
                while True:
                    print(
                        "============ Move the robot end factor to a desired corner coordinate."
                    )
                    input(
                        "============ Press `Enter` when finished..."
                    )
                    P = tutorial.move_group.get_current_pose().pose
                    S = tutorial.move_group.get_current_joint_values()
                    print("============ End Factor Pose:")
                    print("{}".format(P))

                    go_next=input("============ Record this as corner {}?(Y/N)".format(i))
                    if go_next in ['Y','y']:
                        corners.append(P)
                        corner_joint_values.append(S)
                        print('============ Corner {} recorded.'.format(i))
                        break

            fn = input('Please input the path to the .pkl file to save the corners:')
            
            with open(fn,'wb') as f:
                pkl.dump({'corner_poses':corners,'corner_joint_values':corner_joint_values},f)

            print('Corner coordinates are saved to {}'.format(fn))


        corner_pos = np.array([[c.position.x,c.position.y,c.position.z] for c in corners])


        grid_d = float(input('Please input the distance between grid points in meters(decimal numbers accepted):'))
        print('Generating grid waypoints')
        
        aq_region = AcquisitionRegion(corner_pos)
        grid = aq_region.grid(grid_d)

        initial_wp = corners[0]
        waypoints = [copy.deepcopy(initial_wp) for _ in range(len(grid))]
        # The waypoints all have the same orientation as the first corner.
        # The positions of the waypoints are determined by the grid variable.
        for i in range(len(waypoints)):
            waypoints[i].position.x = grid[i,0]
            waypoints[i].position.y = grid[i,1]
            waypoints[i].position.z = grid[i,2]  
       
        print('Waypoint generation done. {} waypoints are generated.'.format(len(waypoints)))

        move_group = tutorial.move_group
        for i in range(len(waypoints)):
            back = input("============ Press `Enter` to move to waypoint {}.".format(i))
            if back == 'b':
                move_group.go(corner_joint_values[0], wait=True)
                move_group.stop()
            else:
                curr_pose = move_group.get_current_pose().pose
                target_pose = copy.deepcopy(curr_pose)

                target_pose.position = waypoints[i].position

                plan,_ = move_group.compute_cartesian_path([curr_pose,target_pose],0.01,0)
                move_group.execute(plan,wait=True)
                move_group.stop()


        print("============ All waypoints are visited! The program is completed.")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
