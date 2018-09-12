# Turtlebot3 3D-SLAM using RTAB-Map with Jetson TX2

![tb3_slam_3d](https://user-images.githubusercontent.com/20625381/43825668-4e781b04-9b30-11e8-828e-3c572ff79b26.jpg)


## Quick Start:

```
mkdir catkin_ws/src -p
cd catkin_ws
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d/master/.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src
catkin build turtlebot3_slam_3d
```

### Launching Demo Program

Download one of the bag files:
```sh
# Small version (300MB)
https://drive.google.com/a/robotis.com/uc?export=download&confirm=SLgY&id=1sfMhQV5ipJm0ghrvQ8HpOw2tTr179aiP

# Full version (1.2GB)
https://drive.google.com/a/robotis.com/uc?export=download&confirm=BkOw&id=1BUQdcuxshEC-W6O9Jkel6sUP9-FxkLca
```

Setup the environment with:
```
cd catkin_ws
source devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/src
```

Then, launch the sample mapping demo:
```
roslaunch turtlebot3_slam_3d demo_bag.launch bag_file:=/path/to/bag_file.bag
```

Default `bag_file` path is set to `turtlebot3_slam_3d/rtab_bag.bag`.
Map is saved to `~/.ros/rtabmap.db` as default setting.


## Run with Turtlebot3

### Hardware Components:
Main Components:
- Turtlebot3 Waffle
- Jetson TX2
- ZED Mini or Intel RealSense D435

Other Components:
- 19V Battery for Jetson TX2
- USB Hub with micro-B connector
- 3 Additional Waffle Plates
- 9 Additional Plate Supports M3x45
- M3x8 Bolts
- Camera Stand

### Software Components:

#### ZED Mini Software Setup
Download ZED SDK for Jetson TX2 from https://www.stereolabs.com/developers/.
Then, build the ROS wrapper with:
```
cd catkin_ws
catkin build zed_wrapper
```

#### Intel RealSense D435 Software Setup
Run the installation script from [jetsonhacks](https://github.com/jetsonhacks/buildLibrealsense2TX) to install the RealSense SDK on Jetson TX2:
```
cd catkin_ws/src/buildLibrealsense2TX
./installLibrealsense.sh 
```
Then, build the ROS wrapper with:
```
cd catkin_ws
catkin build realsense2_camera
```

#### Launch Files:

First, setup the environment:
```
cd catkin_ws
source devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/src
```

To create your own map, run:
```
roslaunch turtlebot3_slam_3d turtlebot3_rtabmap.launch
```

To navigate through your map use:
```
roslaunch turtlebot3_slam_3d turtlebot3_rtabmap.launch localization:=true
```

To use **Intel RealSense D435**, use the following instead:
```
roslaunch turtlebot3_slam_3d turtlebot3_rtabmap.launch use_zed:=false
roslaunch turtlebot3_slam_3d turtlebot3_rtabmap.launch use_zed:=false localization:=true
```
