#! /usr/bin/env python

import roslib
roslib.load_manifest('eye_tracking_server')
import rospy
import actionlib
from controller import AcquisitionControl

import pickle as pkl
import numpy as np
import copy


from region import AcquisitionRegion

# The eye_tracking_server.msg is placed under /catkin_ws/devel/shared/eye_tracking_server/msgs
from eye_tracking_server.msg import GoToAction

from eye_tracking_server.srv import nbOfPosition,nbOfPositionResponse


class GoToServer:
  def __init__(self,grid_d):
    
    # Initialize the waypoints to go to.

    # Assume the corners are already stored.
    fn = input('Please input the path to the .pkl file storing the corners:')
    with open(fn,'rb') as f:
        corners = pkl.load(f)
    print('Corner coordinates are loaded from {}'.format(fn))

    corner_pos = np.array([[c.position.x,c.position.y,c.position.z] for c in corners])
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

    self.waypoints = waypoints
    self.initial_wp = initial_wp  

    # Initialized the robot controller
    self.controller = AcquisitionControl()

    # Initialize the action server.

    self.server = actionlib.SimpleActionServer('GoTo', GoToAction, self.goto, False)
    self.server.start()

    # Initialize the number of locations server.
    s = rospy.Service('nbOfPosition', nbOfPosition, self.handle_nbOfPosition)


  def handle_nbOfPosition(self,req):
      return nbOfPositionResponse(len(self.waypoints))

  def goto(self, goal):
    idx = goal.waypoint_idx
    move_group = self.controller.move_group
    print('Go to idx:{}'.format(idx))

    if idx>=len(self.waypoints):
      self.server.set_aborted()
    else:
      move_group.set_pose_target(self.waypoints[idx])
      success = move_group.go(wait=True)
      move_group.stop()

      if success:
        self.server.set_succeeded()
      else:
        self.server.set_aborted()

if __name__ == '__main__':
  rospy.init_node('GoToServer')
  grid_d = 0.05
  server = GoToServer(grid_d)
  rospy.spin()