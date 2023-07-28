# marker_tracking
The ROS 1 repository for automatic marker tracking using a robotic arm.

# Installation
Clone this package to the src/ folder of your catkin workspace, then build the package.

```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/lina-robotics-lab/marker_tracking.git
  $ cd ~/catkin_ws
  $ rosdep install --from-paths src --ignore-src -r -y
  $ catkin build marker_tracking
```

# Step 1: bring up the robot
First, bring up the real robot or simulated robot, then run the MoveIt! interface.

```bash
cd ${pkg_folder}/scripts/run_server_2.bash
```
You might need to click enter three times after each time the output finishes to pop out.

# Step 2: run the robot arm server and camera.

Source the ROS workspace, then run 

```bash
  $ roslaunch marker_tracking marker_tracking_control.launch
```

You might need to press Enter to start the robot arm control server.

# Step 3 start the tracking control

```bash
  $ rosrun marker_tracking track_marker.py
```

