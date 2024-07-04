
# Collaborative SLAM using multiple robots

This projects aims to generate a single map using multiple robots.

## To deploy the project :

Make sure to install ubuntu 22.04 and [install ros2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) in the system.

Create the workspace and build it. 

```bash
  cd
  mkdir -p ros2_ws/src
  cd ros2_ws
  colcon build --symlink-install
```

Clone the repository into local system

```bash
  cd ~/ros2_ws/src
  git clone https://github.com/V16nesh/multi_robot_system.git
  git clone https://github.com/abdulkadrtr/mapMergeForMultiRobotMapping-ROS2.git
  cd ..
  colcon build --symlink-install
```
Install the neccessary dependencies

```bash
  cd ~/ros2_ws
  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
```
Source the workspace

```bash
  source ~/ros2_ws/install/setup.bash
```

To launch the single robot simulation environment, 
```bash
  ros2 launch multi_robot_system single_robot_sim.launch.py 
```
Use Xterm window to move the robot.

To launch the single robot simulation environment along with SLAM, 
```bash
  ros2 launch multi_robot_system single_robot_slam.launch.py 
```
To launch the two robot simulation environment with map merging, 
```bash
  ros2 launch multi_robot_system multi_spawn.launch.py
```
