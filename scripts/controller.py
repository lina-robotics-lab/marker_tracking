#! /usr/bin/env python3
from __future__ import print_function

import moveit_commander
from geometry_msgs.msg import PoseStamped
from eye_tracking_server.msg import GoToPoseAction, GoToPoseGoal
from collections import deque

import numpy as np

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


class RobotController(object):
    def __init__(self):
        
        self.i = 0

        self.moveit_controller = AcquisitionControl()

        self.client = actionlib.SimpleActionClient('GoTo', GoToPoseAction)
        # self.client.wait_for_server()
        rospy.loginfo("Action server ready")

        self.curve = None
        self.newMats = None

        self.motion_planner = 'bezier' # bezier or moveit

        self.tf = tf.TransformListener(True, rospy.Duration(10.))
        self.goal_pose_listener = rospy.Subscriber('camera/goal_pose', PoseStamped)
        

        self.tf.waitForTransform('tool0', 'base', time=rospy.Time(), timeout=rospy.Duration(10.))
        print('all frames', self.tf.getFrameStrings())

        self.goal_pose_stack = deque(maxlen=10)


    def goal_pose_cbk(self, data):
        self.goal_pose_stack.append(data)

    def get_camera_pose(self):
        (translation, quaternion) = self.tf.lookupTransform('camera', 'base', rospy.Time())
        r = R.from_quat(quaternion)
        rotation_matrix = r.as_matrix()
        return translation, rotation_matrix

    # def transfer_camera_to_tool(self, marker_pos_measure, marker_rotmat_measure):
    #     # transfer measured left handed to camera coor right handed
    #     marker_rotmat_camera = deepcopy(marker_rotmat_measure)
    #     marker_rotmat_camera[1] = -1. * marker_rotmat_camera[1]
    #     marker_pos_camera = marker_pos_measure
    #     # transfer camera coor to tool coor, bias on the rack
    #     marker_pos_tool = deepcopy(marker_pos_camera) + np.array([0., -0.057, 0.046])
    #     # marker_pos_tool[1] = -1. * marker_pos_tool[1]
    #     marker_rotmat_tool = deepcopy(marker_rotmat_camera)
    #     # marker_rotmat_tool[:, 1] = -1 * marker_rotmat_tool[:, 1]
    #     return marker_pos_tool, marker_rotmat_tool

    # def transfer_camera_to_base(self, position, rotmat):
    #     marker_rotmat_camera = rotmat
    #     marker_pos_camera = position
    #     (camera_pos_base, camera_rotmat_base) = self.get_camera_position()
    #     marker_pos_base = np.matmul([marker_pos_camera], marker_rotmat_camera).squeeze() + camera_pos_base
    #     marker_rotmat_base = np.matmul(camera_rotmat_base, marker_rotmat_camera)
    #     return marker_pos_base, marker_rotmat_base

    # def transfer_camera_to_base(self, marker_pos_camera, marker_rotmat_camera):
    #     # marker_rotmat_camera = rotmat
    #     # marker_pos_camera = position
    #     # tool_pose_base = self.get_tool_pose()
    #     marker_pos_tool, marker_rotmat_tool = self.transfer_camera_to_tool(marker_pos_camera, marker_rotmat_camera)
    #     (tool_pos_base, tool_rotmat_base) = self.get_tool_pose()

    #     marker_pos_base = np.matmul([marker_pos_tool], tool_rotmat_base).squeeze() + tool_pos_base
    #     marker_rotmat_base = np.matmul(marker_rotmat_tool, tool_rotmat_base)
    #     # transfer camera rot and pos to tool frame
    #     # marker_rotmat_base[1] = -1. * marker_rotmat_base[1]
    #     return marker_pos_base, marker_rotmat_base

    # def plot_traj(self):
    #     pos_base = []
    #     rotvec_base = []
    #     if self.curve is not None:
    #         for i in range((len(self.curve))):
    #             pos, rotmat = self.transfer_camera_to_base(self.curve[i], self.newMats[i])
    #             pos_base.append(pos)
    #             r = R.from_matrix(rotmat)
    #             rotvec_base.append(r.as_rotvec())
    #     rotvec_base = np.array(rotvec_base)
    #     pos_base = np.array(pos_base)
    #     fig1 = plt.figure(figsize=(4, 4))
    #     ax1 = fig1.add_subplot(111, projection='3d')
    #     # ax1.scatter(x, y, z, c='black')
    #     curve_ar = np.array(self.curve_base)
    #     ax1.scatter(curve_ar[:, 0], curve_ar[:, 1], curve_ar[:, 2], c='blue')
    #     # print(self.curve)
    #     ax1.plot(curve_ar[:, 0], curve_ar[:, 1], curve_ar[:, 2])

    #     ax1.quiver(curve_ar[:, 0], curve_ar[:, 1], curve_ar[:, 2], 
    #         rotvec_base[:, 0], rotvec_base[:, 1], rotvec_base[:, 2], length=0.01, normalize=False)
    #     plt.show()


    # def set_traj(self, curve, newMats):

    #     self.curve = curve / 1000.
    #     self.newMats = newMats
    #     self.curve_base = []
    #     self.newMats_base = []
    #     for i in range((len(self.curve))):
    #         pos, rot = self.transfer_camera_to_base(self.curve[i], self.newMats[i])
    #         self.curve_base.append(pos)
    #         self.newMats_base.append(rot)

    #     print(self.newMats_base[-1])

    
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

    def manual_goto(self, pos, quat):
        rospy.loginfo('manual goal set %s' % pos)
        goal = GoToPoseGoal()
        goal.position_x = pos[0]
        goal.position_y = pos[1]
        goal.position_z = pos[2]
        goal.orientation_x = quat[0]
        goal.orientation_y = quat[1]
        goal.orientation_z = quat[2]
        goal.orientation_w = quat[3]
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
    rospy.init_node('controller_test')
    controller = RobotController()
    pos = [0., 0., 0.05]
    quat = [0., 0., 0., 1.,]
    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = controller.tf.getLatestCommonTime('tool0', 'base_link')
    goal.header.frame_id = "tool0"

    goal.pose.position.x = 0.
    goal.pose.position.y = 0.
    goal.pose.position.z = 0.1

    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]
    # print(controller.tf.getFrameStrings())
    print(goal)
    goal_pose_base = controller.tf.transformPose('base_link', goal)
    goal_pos_base = [goal_pose_base.pose.position.x,
                     goal_pose_base.pose.position.y,
                     goal_pose_base.pose.position.z]
    goal_quat_base = [goal_pose_base.pose.orientation.x,
                      goal_pose_base.pose.orientation.y,
                      goal_pose_base.pose.orientation.z,
                      goal_pose_base.pose.orientation.w]
    # print(goal_pos_base, goal_quat_base)
    controller.manual_goto(goal_pos_base, goal_quat_base)