#! /usr/bin/env python3

import roslib
roslib.load_manifest('eye_tracking_server')
import rospy
import actionlib
from controller import AcquisitionControl
import geometry_msgs
from time import sleep

import rospkg
PACKAGE_HOME = rospkg.RosPack().get_path('eye_tracking_server')

import pickle as pkl
import numpy as np
import copy

from moveit_msgs.msg import Constraints, OrientationConstraint, DisplayTrajectory
from geometry_msgs.msg import Pose



from region import AcquisitionRegion

# The eye_tracking_server.msg is placed under /catkin_ws/devel/shared/eye_tracking_server/msgs
from eye_tracking_server.msg import GoToPoseAction, GoToPoseActionResult

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

def pose2array(pose):
   return np.array([pose.position.x,pose.position.y,pose.position.z])
   
class GoToServer:
  def __init__(self,grid_d):
	
    # Initialized the robot controller
    self.controller = AcquisitionControl()
    self.scene = self.controller.scene

    # Initialize the action server.
    self.server = actionlib.SimpleActionServer('GoTo', GoToPoseAction, self.goto, False)
    self.server.start()

    # Initialize the system settings required by get_key()
    self.key_settings = termios.tcgetattr(sys.stdin)

    self.manual_control_on = False

    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=20,
        )

    self.display_trajectory_publisher = display_trajectory_publisher

    # Adding the box representing the tablet.
    # self.add_box()
    # self.attach_box()
    self.add_table()


  def fix_eff_orientation_in_planning(self):
    self.controller.move_group.clear_path_constraints()
    curr_pose = self.controller.move_group.get_current_pose().pose


    # Adding the orientation constraint for the end factor.
    # We want the end factor to always face the same orientation as the initial waypoint.
    ocm = OrientationConstraint()
    ocm.link_name = self.controller.eef_link
    ocm.header.frame_id = self.controller.planning_frame 
    ocm.orientation.x = curr_pose.orientation.x
    ocm.orientation.y = curr_pose.orientation.y
    ocm.orientation.z = curr_pose.orientation.z
    ocm.orientation.w = curr_pose.orientation.w

    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    constraints = Constraints()
    constraints.orientation_constraints.append(ocm)

    self.controller.move_group.set_path_constraints(constraints)

  def spin(self):
    self.stop()
    input('Press Enter to start the server.')
    print('The application is now running in server mode. Press "m" to enter manual mode. Press Ctrl+C to shutdown the server.')    
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
          self.detach_box()
          break

  def manual_control(self):
    '''
      The keyboard interaction interface when the server is running in manual mode.
    '''
    print('Move down (d), or exit(e) manual mode?')
    while(1):
      command = get_key(self.key_settings)
      if command == 'e':
        print('Exit manual mode and resuming server mode. Press "m" to enter manual mode again.')
        self.manual_control_on = False  
        break
      elif not command in ['d', '']:
        print('Command {} not recognized.'.format(command))
      elif command == 'd':
        success = self.move_down()

  def handle_nbOfPosition(self,req):
      return nbOfPositionResponse(len(self.waypoints))
  
  def stop(self):
    self.controller.move_group.stop()

  def __gotopose(self,target_pose):
    move_group = self.controller.move_group
    curr_pose = move_group.get_current_pose().pose
    target_pose.orientation = curr_pose.orientation

    # We do not want to change the orientation of the target_pose.

    # The current pose MUST NOT be included in the waypoints.
    
    # self.fix_eff_orientation_in_planning()

    plan,_ = move_group.compute_cartesian_path([target_pose],0.005,0)
    success = move_group.execute(plan,wait=True)
    move_group.stop()
    
    # Ensure the target location is really reached.
    try:
    	total_tries = 10
    	for _ in range(total_tries):
    	     curr_pose = move_group.get_current_pose().pose
    	     
    	     tolerance = 0.1
    	     
    	     dist = np.linalg.norm(pose2array(curr_pose)-pose2array(target_pose))
    	     print('Distance to  target pose:{}'.format(dist))
    	     if dist<tolerance:
    	        break
    	     sleep(0.5)
    	     if _ == total_tries-1:
                print('Target not reached')   
    except KeyboardInterrupt:
    	print('Keyboard interrupt detected.')
    
    return success
  
  def move_down(self):
    # in manual mode
    move_group = self.controller.move_group
    current_pose = self.controller.move_group.get_current_pose().pose
    print(current_pose)
    current_pose.position.z += 0.05
    move_group.set_pose_target(current_pose)
    # plan_results = move_group.plan()
    # print('plan_successful: {}'.format(plan_results[0]))
    # keys = input("Press Enter to execute the plan, or r to replay the planned path, or a to abort")
    # if keys == "":
    #   success = move_group.execute(plan_results[1], wait=True)
    # # elif input == "r":
    # #   move_group.
    # elif keys == "a":
    #   success = False
    success = move_group.go(wait=True)
    
    return success

  def goto(self, goal, cartisian=True):

    '''
      When the server is not running in manual mode, it accepts action requests from the client
      through the GoTo action server.
    '''
    if self.manual_control_on:
      print('Request received but currently manual control is active. Not responding.')
      self.server.set_aborted()
    else:
      move_group = self.controller.move_group

      success = False
      result = GoToPoseActionResult()
      print(goal)
      if cartisian:
        waypoints = goal.goal_poses.poses
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        rospy.loginfo('planning success')
        self.display_trajectory(plan)
        success = move_group.execute(plan, wait=True)
        rospy.loginfo('move_success %s' % success)
        # print("Press g to execute the plan, or r to replay the planned path, or a to abort")
        # while True:
        #   key = get_key(self.key_settings)
        #   if key == "g":
        #     success = move_group.execute(plan, wait=True)
        #     break
        #   elif input == "r":
        #     self.display_trajectory(plan)
        #   elif key == "a":
        #     success = False
        #     print('Aborted')
        #     break
        #   else:
        #      print('Command not recognized')
        #      pass

      else:
        raise NotImplementedError
        # move_group.set_pose_target(goal.goal_poses)
        # plan_results = move_group.plan()
        # print('plan_successful: {}'.format(plan_results[0]))
        # keys = input("Press Enter to execute the plan, or r to replay the planned path, or a to abort")
        # if keys == "":
        #   success = move_group.execute(plan_results[1], wait=True)
        # elif input == "r":
        #   move_group.display_trajectory()
        # elif keys == "a":
        #   success = False
        # success = move_group.go(wait=True)
      
      if success:
        result.result.waypoint_reached = True
        self.server.set_succeeded(result.result)
        rospy.loginfo('set success')
      else:
        self.server.set_aborted()

  def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.controller.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

          
  def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.controller.box_name
        scene = self.controller.scene

        drive_name = 'drive'

        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.controller.planning_frame
        box_pose.pose = self.controller.move_group.get_current_pose().pose

        ## Add the drive box
        drive_pose = geometry_msgs.msg.PoseStamped()
        drive_pose.header.frame_id = self.controller.planning_frame
        drive_pose.pose = self.controller.move_group.get_current_pose().pose


        ############### The far corners configuration ###############
        box_pose.pose.position.y = box_pose.pose.position.y # + 0.08    


        drive_pose.pose.position.y = drive_pose.pose.position.y - 0.055
        drive_pose.pose.position.z = drive_pose.pose.position.z - 0.08
        

        scene.add_box(box_name, box_pose, size=(0.2, 0.2, 0.15))
        # scene.add_box(drive_name, drive_pose, size=(0.09, 0.02, 0.02))
        



        ############## The close corners configuration###############
        # box_pose.pose.position.y = box_pose.pose.position.y - 0.05 # This is to place the tablet a few centimeters away from the actual flange.        
        # scene.add_box(box_name, box_pose, size=(0.5, 0.5, 0.02))

        
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  def add_table(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    table_name = 'table'
    scene = self.controller.scene

    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = self.controller.planning_frame
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.z = -1.01

    table_pose.pose.position.y = 0.90
    scene.add_box(table_name, table_pose, size=(2, 2, 2))

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
      box_name = self.controller.box_name
      drive_name = 'drive'
      robot = self.controller.robot
      scene = self.controller.scene
      eef_link = self.controller.eef_link
      group_names = self.controller.group_names

      touch_links = ['wrist_1_link','wrist_2_link','wrist_3_link'] 
      # The box is allowed to touch the three wrist links, and not allowed to touch the remaining links, like forarm_link
      

      # touch_links = [] # The box is not allowed to touch any links.
      scene.attach_box(eef_link, box_name, touch_links=touch_links)
      # scene.attach_box(eef_link, drive_name, touch_links=[])


      return self.wait_for_state_update(
          box_is_attached=True, box_is_known=False, timeout=timeout
      )

  def detach_box(self, timeout=4):
     
      box_name = self.controller.box_name
      scene = self.controller.scene
      eef_link = self.controller.eef_link

    
      scene.remove_attached_object(eef_link, name=box_name)
      
      return self.wait_for_state_update(
          box_is_known=True, box_is_attached=False, timeout=timeout
      )

  def remove_box(self, timeout=4):
     
      box_name = self.controller.box_name
      scene = self.controller.scene

     
      scene.remove_world_object(box_name)

      return self.wait_for_state_update(
          box_is_attached=False, box_is_known=False, timeout=timeout
      )
  def wait_for_state_update(
      self, box_is_known=False, box_is_attached=False, timeout=4
  ):
      box_name = self.controller.box_name
      scene = self.controller.scene

    
      start = rospy.get_time()
      seconds = rospy.get_time()
      while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = scene.get_attached_objects([box_name])
          is_attached = len(attached_objects.keys()) > 0

          # Test if the box is in the scene.
          # Note that attaching the box will remove it from known_objects
          is_known = box_name in scene.get_known_object_names()

          # Test if we are in the expected state
          if (box_is_attached == is_attached) and (box_is_known == is_known):
              return True

          # Sleep so that we give other threads time on the processor
          rospy.sleep(0.1)
          seconds = rospy.get_time()

      # If we exited the while loop without returning then we timed out
      return False
      ## END_SUB_TUTORIAL
if __name__ == '__main__':
  rospy.init_node('GoToGoalPoseServer')
  grid_d = 0.05
  
  server = GoToServer(grid_d)
  server.spin()
