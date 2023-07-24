#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from controller import AcquisitionControl

import moveit_commander
import numpy as np
from copy import deepcopy

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

def get_tool_pose(pose):
        position = [pose.position.x,
                    pose.position.y,
                    pose.position.z]
        r = R.from_quat([pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w])
        orientation = r.as_matrix()
        return position, orientation

def transfer_camera_to_tool(marker_pos_camera, marker_rotmat_camera):
    marker_pos_tool = deepcopy(marker_pos_camera)
    marker_pos_tool[1] = -1. * marker_pos_tool[1]
    marker_rotmat_tool = deepcopy(marker_rotmat_camera)
    marker_rotmat_tool[:, 1] = -1 * marker_rotmat_tool[:, 1]
    return marker_pos_tool, marker_rotmat_tool


def transfer_camera_to_base(camera_pose_base, marker_pos_camera, marker_rotmat_camera):
    # marker_rotmat_camera = rotmat
    # marker_pos_camera = position
    marker_pos_tool, marker_rotmat_tool = transfer_camera_to_tool(marker_pos_camera, marker_rotmat_camera)
    (tool_pos_base, tool_rotmat_base) = get_tool_pose(camera_pose_base)
    marker_pos_base = np.matmul([marker_pos_tool], marker_rotmat_tool).squeeze() + tool_pos_base
    marker_rotmat_base = np.matmul(marker_rotmat_tool, tool_rotmat_base)
    return marker_pos_base, marker_rotmat_base

def talker():
    pub = rospy.Publisher('eef_pose', Pose, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True, disable_signals=True)
    controller = AcquisitionControl()
    rate = rospy.Rate(10) # 10hz
    while True:
        try:
            msg = controller.move_group.get_current_pose().pose
            camera_origin_pos_base, camera_origin_rotmat_base = transfer_camera_to_base(msg, np.zeros([3,]), np.eye(3))
            # position = [msg.position.x, msg.position.y, msg.position.z]
            # orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            # r = R.from_matrix(camera_origin_rotmat_base)
            rospy.loginfo("camera_origin_pos_base %s " % camera_origin_pos_base)
            rospy.loginfo("camera_origin_rotmat_base %s" % camera_origin_rotmat_base)
            # rospy.loginfo("pos %s " % position)
            # pub.publish(msg)
            rate.sleep()
        except KeyboardInterrupt:
            print("keyboard interrupt")
            break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass