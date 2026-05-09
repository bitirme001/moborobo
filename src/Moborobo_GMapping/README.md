# Moborobo_SLAM

### To access the last year's work: 
https://github.com/kaantuncer/Moborobo-Project/
### Important! If you face cmake errors while building, install the package that written in the log
## Also Orin cannot booted with SD card since it does not have jetpack image
## Which changes are made?


#### FOR RSLIDAR:

* Run these commands in your workspace:
* ```
  git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
  cd rslidar_sdk
  git submodule init
  git submodule update

  sudo apt-get update
  sudo apt-get install -y libyaml-cpp-dev
  sudo apt-get install -y  libpcap-dev  
  ```
* For more info, you can visit: https://github.com/RoboSense-LiDAR/rslidar_sdk
  
#### FOR MOBOROBO GMAPPING:

* rewriten line 17 in navigation.launch (node pkg gmapping part)
* removed robosense_16.dae geometry part in RS-16.urdf.xacro file (line 20-22)
* for rslidar we set the parameters for ethernet connection as:
  * connect the ethernet (yellow thick cable)
  * enter the wired settings from top-right menu
  * set the ipv4 method as manual
  * enter address as 192.168.1.102
  * enter netmask as 255.255.255.0
* some parameters are changed and added to the config.yaml file such as:
  * change-> send_packet_ros: true (line 6)
  * change-> lidar_type: RS16 (line 10)
  * add-> frame_id: rslidar (line 13)
    
#### FOR MOBOROBO SLAM WITH "slam-toolbox":
##### On robot:
* `sudo apt install ros-noetic-slam-toolbox`
* Download the zip file in the link: `https://github.com/SteveMacenski/slam_toolbox/tree/noetic-devel`.
* You need to extract the zip file inside the on_robot/src directory

* `sudo apt install ros-noetic-pointcloud-to-laserscan`
* Run this command in your workspace/src (on_robot/src): `git clone https://github.com/ros-perception/pointcloud_to_laserscan.git --branch indigo-devel --single-branch`
* Go to the file Moborobo-Project/on_robot/src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_nodelet.cpp and change the line 241 as : `PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet);`  (inspired from https://github.com/ros-perception/depthimage_to_laserscan/issues/31)

##### Simulation:
* commented out line 17-23 in navigation.launch (node pkg gmapping part)

  * ##### For execution:
    * `sudo apt install ros-noetic-slam-toolbox`
    * `cp /opt/ros/noetic/share/slam_toolbox/config/mapper_params_online_async.yaml catkin_ws/src/moborobo/config/`
    * `roslaunch slam_toolbox online_async.launch params_file:=./src/moborobo/config/mapper_params_online_async.yaml use_sim_time:=true 2> >(grep -v TF_REPEATED_DATA buffer_core)`
   
#### FOR ZED2 CAMERA
* ```
  sudo apt install ros-noetic-rviz-imu-plugin
  sudo apt install ros-noetic-rtabmap
  sudo apt install ros-noetic-rtabmap-ros
  sudo apt install ros-noetic-depthimage-to-laserscan

  cd /your_workspace(on_robot)/src
  git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
  cd ..
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make -DCMAKE_BUILD_TYPE=Release
  source devel/setup.bash

  cd /your_workspace(on_robot)/src
  git clone https://github.com/stereolabs/zed-ros-interfaces.git
  cd ../
  rosdep install --from-paths src --ignore-src -r -y
  ```
* Before `catkin_make` you should remove the zed-ros-interfaces file which is **inside the zed-ros-wrapper** file
* ```
  catkin_make -DCMAKE_BUILD_TYPE=Release
  source devel/setup.bash
  
  cd /your_workspace(on_robot)/src
  git clone https://github.com/stereolabs/zed-ros-examples.git
  cd ..
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make -DCMAKE_BUILD_TYPE=Release
  source devel/setup.bash
  ```
* Run this command to test to see whether it works or not: `roslaunch zed_display_rviz display_zed2.launch`

#### FOR ROAD DETECTION
##### PyTorch Installation:
* Download the file `torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl` from the website https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/
* Run these commands directly
  ```
  export TORCH_INSTALL=~/Downloads/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
  
  pip install --upgrade pip
  pip install numpy==’1.16.5’
  pip install --no-cache https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
  ```

##### MobileSAM
* ```
  cd your_workspace(on_robot)/src/moborobot/scripts
  git clone https://github.com/ChaoningZhang/MobileSAM.git
  ```
* There is a file named *mobile_sam* inside the cloned repository
* Cut the *mobile_sam* folder and paste it to the scripts file (one level higher)
* Run the python script `python3 path/to/python/script/road_detector.py`

#### FOR RUNNING GPS:

* `pip install pyserial`
* `sudo apt install ros-noetic-nmea-msgs`
* Need to be sure that gps is connected with usb port

* ##### if permission denied while running the py script:
  * `sudo su`
  * //type your password
  * `cd /`
  * `cd dev`
  * `chown username ttyUSB0`
 
* `rosrun moborobo locoSysPublisher.py` for publishing the gps data
* `rosrun moborobo locoSysSubscriber.py` for obtaining and reading the data (or you can simply run `rostopic echo /moborobo/mygps`)

Simulation: `sudo apt-get install ros-noetic-hector-gazebo-plugins`


#### PATH PLANNING:



## How to Run?
* run this command in catkin_ws before launch: `source devel/setup.bash`
* for launch: `roslaunch moborobo gmapping_room.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)`

