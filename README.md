# marker_tracking
The ROS 1 repository for automatic marker tracking using a robotic arm.

# Installation
Clone this package to the src/ folder of your catkin workspace, then build the package.

```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/lina-robotics-lab/eye_tracking_server.git
  $ cd ~/catkin_ws
  $ rosdep install --from-paths src --ignore-src -r -y
  $ catkin build eye_tracking_server
```

# Step 1: bring up the robot
First, bring up the real robot or simulated robot, then run the MoveIt! interface.

```bash
cd ${pkg_folder}/scripts/run_server_2.bash
```
You might need to click enter three times after each time the output finishes to pop out.

# Step 2: run the server.

Source the ROS workspace, then run 

```bash
  $ rosrun eye_tracking_server RobotArmGoalPoseServer.py
```


Then the application will be up and ready to receive requests from the client.

The application can operate under two modes: server mode and manual mode. By default, the application will be running in server mode. Under this mode, the application only serves as a server that interacts with the client via ROS. 

By pressing 'm' in server mode, the application will switch to manual mode. Under this mode, the application no longer accepts requests from the client, but directly interacts with the user via keyboard commands. Follow the prompt to control the robot to go to different waypoints or corners. Press 'e' to exit the manual mode and return to server mode.

# Step 3 run the camera 

```bash
  $ rosrun eye_tracking_server percept_and_publish.py
```

