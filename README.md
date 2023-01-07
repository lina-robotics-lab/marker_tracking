# eye_tracking_server_server
The ROS 1 repository for automatic eye tracking acquisition using a robotic arm.

# How to use this package
Clone this package to the src/ folder of your catkin workspace, then build the package.

```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/lina-robotics-lab/eye_tracking_server.git
  $ cd ~/catkin_ws
  $ rosdep install --from-paths src --ignore-src -r -y
  $ catkin build eye_tracking_server
```
