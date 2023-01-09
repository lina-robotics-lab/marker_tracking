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


# Modules required by the get_key() function, used in the manual mode.
import os
import select
import sys
import termios
import tty

def get_key(settings):
  tty.setraw(sys.stdin.fileno())
  rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
  if rlist:
    key = sys.stdin.read(1)
  else:
    key = ''

  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

class GoToServer:
  def __init__(self,grid_d):
    
    # Initialize the waypoints to go to.

    # Assume the corners are already stored.
    fn = input('Please input the path to the .pkl file storing the corners:')
    with open(fn,'rb') as f:
        data = pkl.load(f)
        corners = data['corner_poses']
    print('Corner coordinates are loaded from {}'.format(fn))
    print(corners)

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
    self.corners = corners

    # Initialized the robot controller
    self.controller = AcquisitionControl()

    # Initialize the action server.

    self.server = actionlib.SimpleActionServer('GoTo', GoToAction, self.goto, False)
    self.server.start()

    # Initialize the number of locations server.
    s = rospy.Service('nbOfPosition', nbOfPosition, self.handle_nbOfPosition)

    # Initialize the system settings required by get_key()
    self.key_settings = termios.tcgetattr(sys.stdin)

    self.manual_control_on = False

  def spin(self):
    self.stop()
    input('Press Enter to start the server.')
    print('Server started. Press "m" to enter manual mode. Press Ctrl+C to shutdown the server.')    
    while(1):
      key = get_key(self.key_settings)
      if key=='m':
        self.stop()
        print('Stopping the robot and entering manual control.')
        self.manual_control_on = True
        self.manual_control()
      else:
        if (key == '\x03'):
          self.stop()
          break
  def manual_control(self):
    '''
      The keyboard interaction interface when the server is running in manual mode.
    '''
    while(1):
      command = input('Go to corner(c), waypoint(w), or exit(e) manual mode?')
      
      if command == 'e':
        print('Exit manual mode. Press "m" to enter manual mode again.')
        self.manual_control_on = False  
        break
      elif not command in ['c','w']:
        print('Command {} not recognized.'.format(command))
        
      elif command == 'c':
        idx = int(input('Input the index of the corner.'))
        self.__gotocorner(idx) 

      elif command == 'w':
          idx = int(input('Input the index of the waypoint.'))
          self.__gotowaypoint(idx)

  def handle_nbOfPosition(self,req):
      return nbOfPositionResponse(len(self.waypoints))
  
  def stop(self):
    self.controller.move_group.stop()

  def __gotopose(self,target_pose):
    move_group = self.controller.move_group
    curr_pose = move_group.get_current_pose().pose
        
    plan,_ = move_group.compute_cartesian_path([curr_pose,target_pose],0.01,0)
    success = move_group.execute(plan,wait=True)
    move_group.stop()

    return success

  def __gotowaypoint(self,idx):
    move_group = self.controller.move_group
    curr_pose = move_group.get_current_pose().pose
    target_pose = copy.deepcopy(curr_pose)
    
    if idx<=len(self.waypoints):
      print('Go to waypoint idx:{}'.format(idx))

      target_pose.position = self.waypoints[idx].position
      success = self.__gotopose(target_pose)
    else:
      print("waypoint index {} out of bounds.".format(idx))
      success = False

    return success

  def __gotocorner(self,idx):
    move_group = self.controller.move_group
    curr_pose = move_group.get_current_pose().pose
    target_pose = copy.deepcopy(curr_pose)
    
    if idx<=len(self.corners):
      print('Go to corner idx:{}'.format(idx))

      target_pose.position = self.corners[idx].position
      success = self.__gotopose(target_pose)
    else:
      print("Corner index {} out of bounds.".format(idx))
      success = False

    return success

  def goto(self, goal):

    '''
      When the server is not running in manual mode, it accepts action requests from the client
      through the GoTo action server.
    '''
    if self.manual_control_on:
      print('Request received but currently manual control is active. Not responding.')
      self.server.set_aborted()
    else:
      idx = goal.waypoint_idx
      move_group = self.controller.move_group
      
      if idx>=len(self.waypoints):
        self.server.set_aborted()
      else:
        # move_group.set_pose_target(self.waypoints[idx])
        # success = move_group.go(wait=True)
        
        success = self.__gotowaypoint(idx)
        if success:
          self.server.set_succeeded()
        else:
          self.server.set_aborted()
        
if __name__ == '__main__':
  rospy.init_node('GoToServer')
  grid_d = 0.04
  server = GoToServer(grid_d)
  server.spin()