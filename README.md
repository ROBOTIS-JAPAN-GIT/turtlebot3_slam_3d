# Turtlebot3 3D-SLAM using RTAB-Map with Jetson TX2

![tb3_slam_3d](https://user-images.githubusercontent.com/20625381/43825668-4e781b04-9b30-11e8-828e-3c572ff79b26.jpg)

Sample repository for creating a three dimensional map of the environment in real-time and navigating through it.
Object detection using YOLO is also performed, showing how neural networks can be used to take advantage of the image database stored by RTAB-Map and use it to e.g. localize objects in the map.

![system_chart](https://user-images.githubusercontent.com/20625381/45400779-10728100-b63d-11e8-8b66-60c1c3080269.png)

## Screencast:

Click for better quality.

[![turtlebot3_slam_3d_mapping](https://user-images.githubusercontent.com/20625381/49206744-58ac6e80-f3f6-11e8-895b-23a05ec03512.gif)](https://drive.google.com/open?id=11fpvLkPqPS2xIGGCNxgHxWuRkkYeu7e-)

Objects found by the robot are mapped as below:

![semantic_map](https://user-images.githubusercontent.com/20625381/49587668-e57f9b00-f9a7-11e8-8414-81462b6e2e36.png)

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

Then, launch the sample mapping demo with the following. It takes a while to load the nodes and to play the bag file.
```
roslaunch turtlebot3_slam_3d demo_bag.launch bag_file:=/absolute/path/to/bag_file.bag
```

Default `bag_file` path is set to `turtlebot3_slam_3d/rtab_bag.bag`.

Map is saved to `~/.ros/rtabmap.db` as default setting.


## Run with Turtlebot3

### Hardware Components:

| | Part name | Quantity |
|:---|:---:|:---:|
| **Main Components** | [Waffle Pi](http://www.robotis.us/turtlebot-3-waffle-pi/) | 1 |
| | [Jetson TX2](https://developer.nvidia.com/embedded/buy/jetson-tx2-devkit) | 1 |
| | [ZED Mini](https://www.stereolabs.com/zed-mini/) or [RealSense D435](https://simplecore.intel.com/realsensehub/wp-content/uploads/sites/63/D435_Series_ProductBrief_010718.pdf) | 1 |
| **Supplementary Items** | [19V Battery for TX2](https://direct.sanwa.co.jp/contents/torisetsu/700-BTL017BK_m.pdf) | 1 |
| | USB Hub (micro-B to Type A) | 1 |
| **Camera Bracket** | [ZED Mini Camera Bracket](https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d/blob/master/meshes/waffle_deep_zed_mini_stand.stl) | 1 |
| | M2x4mm | 2 |
| | M3x8mm | 2 |
| **Chassis Parts** | Waffle Plate | 3 |
| | Plate Support M3x45mm | 11 |
| | M3x8mm | 22 |

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
roslaunch turtlebot3_slam_3d turtlebot3_slam_3d.launch
```

To navigate through your map use:
```
roslaunch turtlebot3_slam_3d turtlebot3_zed_bringup.launch
roslaunch turtlebot3_slam_3d rtabmap.launch localization:=true
```

With **Intel RealSense D435**, use the following instead:
```
roslaunch turtlebot3_slam_3d turtlebot3_slam_3d.launch use_zed:=false
```
```
roslaunch turtlebot3_slam_3d turtlebot3_d435_bringup.launch
roslaunch turtlebot3_slam_3d rtabmap.launch localization:=true use_zed:=false
```
