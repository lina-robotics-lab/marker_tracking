#! /usr/bin/env python3
from __future__ import print_function

import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from std_msgs.msg import Header
from eye_tracking_server.msg import GoToPoseAction, GoToPoseGoal
from collections import deque

import numpy as np
import rospy

import actionlib
from copy import deepcopy

import tf




try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from moveit_commander.conversions import pose_to_list

# Modules required by the get_key() function, used in the manual mode.
import os
import select
import sys
import termios
import tty


## END_SUB_TUTORIAL

def get_key(settings):
  tty.setraw(sys.stdin.fileno())
  rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
  if rlist:
    key = sys.stdin.read(1)
  else:
    key = ''

  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

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


class RobotController(object):
    def __init__(self):
        self.i = 0
        self.client = actionlib.SimpleActionClient('GoTo', GoToPoseAction)
        self.client.wait_for_server()
        rospy.loginfo("Action server ready")

        self.curve = None
        self.newMats = None

        self.motion_planner = 'bezier' # bezier or moveit

        self.tf = tf.TransformListener(True, rospy.Duration(10.))
        # self.goal_pose_listener = rospy.Subscriber('camera/goal_pose', PoseArray)
        

        self.tf.waitForTransform('tool0', 'base', time=rospy.Time(), timeout=rospy.Duration(10.))
        rospy.loginfo('get all tf frames')
        rospy.loginfo(self.tf.getFrameStrings())

        self.goal_pose_stack = deque(maxlen=10)
        self.goal_pose_array = None

    # def goal_pose_cbk(self, data):
    #     self.goal_pose_stack.append(data)

    # def get_camera_pose(self):
    #     (translation, quaternion) = self.tf.lookupTransform('camera', 'base', rospy.Time())
    #     r = R.from_quat(quaternion)
    #     rotation_matrix = r.as_matrix()
    #     return translation, rotation_matrix

    def get_goal_pose_array(self, topic_name):
        """get goal pose array from ros topic topic_name.

        Args:
            topic_name (str): ros topic name to receive goal pose array.
        """
        self.goal_pose_array = rospy.wait_for_message(topic_name, PoseArray, timeout=30.)
    
    def goto_pose_array(self):
        """Go to a pose array received by self.get_goal_pose_array.
        """
        # if self.goal_pose_array is not None:
        #     goal = GoToPoseGoal()
        #     goal.goal_poses.header.seq = 1
        #     goal.goal_poses.header.stamp = rospy.Time().now()
        #     goal.goal_poses.header.frame_id = 'base_link'
        #     goal.goal_poses.poses = self.transform_pose_array(self.goal_pose_array)
        #     self.client.send_goal(goal)
        #     self.client.wait_for_result()
        #     result = self.client.get_result()
        #     return result
        # else:
        #     rospy.loginfo('pose array goal not received.')
        #     return None
        if self.goal_pose_array is not None:
            goal_poses_base_link = self.transform_pose_array(self.goal_pose_array)
            for goal_pose in goal_poses_base_link:
                self.manual_goto(goal_pose)

    def transform_pose_array(self, pose_array, target_frame='base_link'):
        """transform a pose array from current frame to target frame.

        Args:
            pose_array (geometry_msgs/PoseArray): a pose array 
            target_frame (str, optional): target frame. Defaults to 'base_link'.

        Returns:
            Pose[]: A list of poses on target frame.
        """
        assert pose_array.header.frame_id != target_frame
        current_frame = pose_array.header.frame_id
        header = Header()
        header.seq = 1
        header.stamp = self.tf.getLatestCommonTime(current_frame, target_frame)
        header.frame_id = current_frame
        goal_poses = []
        for i in range(len(pose_array.poses)):
            pose_stamped = PoseStamped()
            pose_stamped.header = header
            pose_stamped.pose = pose_array.poses[i]
            goal_poses.append(self.tf.transformPose(target_frame, pose_stamped).pose)
        return goal_poses

        


    def goto(self):
        if self.curve is None:
            pass 
        else: 
            goal_pos, goal_rotmat = self.curve_base[self.i], self.newMats_base[self.i]
            rospy.loginfo('get goal: %s' % goal_pos)
            rospy.loginfo('current pose %s' %self.get_tool_pose()[0] )
            r = R.from_matrix(goal_rotmat)
            orientation = r.as_quat()
            rospy.loginfo('goal quat %s' % orientation)
            goal = GoToPoseGoal()
            goal.position_x = goal_pos[0]
            goal.position_y = goal_pos[1]
            goal.position_z = goal_pos[2]
            goal.orientation_x = orientation[0]
            goal.orientation_y = orientation[1]
            goal.orientation_z = orientation[2]
            goal.orientation_w = orientation[3]
            self.client.send_goal(goal)
            self.client.wait_for_result()
            move_result = self.client.get_result()
            rospy.loginfo('move result "%s"' % move_result)
            # if move_result.waypoint_reached:
            self.i += 1

    def manual_goto(self, pose):
        """manually input goal pose (single) for the robot to go.

        Args:
            pose (geometry_msgs/Pose): goal pose.
        """
        if isinstance(pose, PoseStamped):
            pose = pose.pose
        rospy.loginfo('manual goal set %s' % pose)
        goal = GoToPoseGoal()
        goal.goal_poses.header.seq = 1
        goal.goal_poses.header.stamp = rospy.Time().now()
        goal.goal_poses.header.frame_id = 'base_link'
        goal.goal_poses.poses = [pose]
        self.client.send_goal(goal)
        self.client.wait_for_result()
        move_result = self.client.get_result()
        rospy.loginfo('move result "%s"' % move_result)
        
    def is_moving(self):
        joint_states = rospy.wait_for_message('joint_states')
        velo = joint_states.velocity
        if np.max(np.abs(velo)) > 1e-5:
            return False
        else:
            return True

if __name__ == '__main__':
    import rospy
    import tf
    rospy.init_node('controller_test', disable_signals=True)
    controller = RobotController()
    pos = [0., 0., 0.05]
    quat = [0., 0., 0., 1.,]
    def demo_control_goto(pos, quat):
        goal = PoseStamped()

        goal.header.seq = 1
        goal.header.stamp = controller.tf.getLatestCommonTime('tool0', 'base_link')
        goal.header.frame_id = "tool0"

        goal.pose.position.x = pos[0]
        goal.pose.position.y = pos[1]
        goal.pose.position.z = pos[2]

        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]
        # print(controller.tf.getFrameStrings())
        print(goal)
        goal_pose_base = controller.tf.transformPose('base_link', goal)
        # goal_pos_base = [goal_pose_base.pose.position.x,
        #                 goal_pose_base.pose.position.y,
        #                 goal_pose_base.pose.position.z]
        # goal_quat_base = [goal_pose_base.pose.orientation.x,
        #                 goal_pose_base.pose.orientation.y,
        #                 goal_pose_base.pose.orientation.z,
        #                 goal_pose_base.pose.orientation.w]
        controller.manual_goto(goal_pose_base.pose)
    print("press w/a/s/d for moving the tool, and press i/j/k/l to tilt the tool")
    while not rospy.is_shutdown():
        try:
            key = get_key(termios.tcgetattr(sys.stdin))
            if key == 'w':
                demo_control_goto([0.05, 0., 0.], [0., 0., 0., 1.,])
            elif key == 's':
                demo_control_goto([-0.05, 0., 0.], [0., 0., 0., 1.,])
            elif key == 'a':
                demo_control_goto([0., 0.05, 0.], [0., 0., 0., 1.,])
            elif key == 'd':
                demo_control_goto([0., -0.05, 0.], [0., 0., 0., 1.,])
            elif key == 'i':
                quat = tf.transformations.quaternion_about_axis(10./180.*np.pi, (1,0,0))
                demo_control_goto([0., 0., 0.], quat)
            elif key == 'k':
                quat = tf.transformations.quaternion_about_axis(-10./180.*np.pi, (1,0,0))
                demo_control_goto([0., 0., 0.], quat)
            elif key == 'j':
                quat = tf.transformations.quaternion_about_axis(10./180.*np.pi, (0,1,0))
                demo_control_goto([0., 0., 0.], quat)
            elif key == 'l':
                quat = tf.transformations.quaternion_about_axis(-10./180.*np.pi, (0,1,0))
                demo_control_goto([0., 0., 0.], quat)
            elif key == '\x03':
                break

        except KeyboardInterrupt:
            break
            