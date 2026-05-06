# Autonomous Smart Waste Collection Robot

This project aims to develop an autonomous mobile robot system for efficient waste collection on campus environments using ROS Noetic.

---

## Features

- Differential drive mobile robot control (Roboteq motor driver)
- Odometry estimation
- RoboSense RS-LiDAR 16 integration
- PointCloud2 → LaserScan conversion
- SLAM mapping using slam_toolbox
- Map saving using map_server

---

## System Architecture

```text
Joystick / cmd_vel
        ↓
kinematics_node
        ↓
motor_commands
        ↓
roboteq_driver
        ↓
Robot movement

LiDAR (rslidar_points)
        ↓
pointcloud_to_laserscan
        ↓
/scan
        ↓
slam_toolbox
        ↓
/map

---


## Step-by-Step Run Commands

---

###  1. Start ROS Master

```bash
roscore

2. Start Robot Motor and Odometry
cd ~/moborobo_ws
source devel/setup.bash
sudo chmod 666 /dev/ttyACM0
roslaunch moborobot motor_only.launch

Check odometry:

rostopic echo /odom

---

3. Start RoboSense LiDAR
sudo ifconfig enp0s31f6 192.168.1.102 netmask 255.255.255.0 up

cd ~/moborobo_ws
source devel/setup.bash
roslaunch rslidar_pointcloud rs_lidar_16.launch

Check LiDAR topic:

rostopic hz /rslidar_points

---

4. Static TF Setup

Check LiDAR frame:

rostopic echo -n 1 /rslidar_points/header

---

If the frame is rslidar, run:

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link rslidar 100

---

Add base_footprint frame:

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link base_footprint 100

---

5. Convert PointCloud2 to LaserScan
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node \
cloud_in:=/rslidar_points \
scan:=/scan \
_target_frame:=base_link \
_min_height:=-1.0 \
_max_height:=1.0 \
_range_min:=0.3 \
_range_max:=30.0 \
_transform_tolerance:=0.5

---

Check scan topic:

rostopic hz /scan
6. Start SLAM Toolbox
rosparam delete /slam_toolbox

---


If this gives an error, it can be ignored.

roslaunch slam_toolbox online_sync.launch \
scan_topic:=/scan \
base_frame:=base_footprint \
odom_frame:=odom \
map_frame:=map \
use_sim_time:=false

---


Check map topic:

rostopic hz /map
7. Visualize in RViz
rviz

---

RViz settings:

Fixed Frame: map
Add → Map → /map
Add → LaserScan → /scan
Add → TF
8. Save the Map
rosrun map_server map_saver -f ~/moborobo_map

---

Generated files:

~/moborobo_map.pgm
~/moborobo_map.yaml
9. Open Saved Map
eog ~/moborobo_map.pgm

---





