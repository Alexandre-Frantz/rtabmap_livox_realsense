# rtabmap_livox_realsense
ROS 2 package that launches SLAM using RTAB-Map using a Gazebo [Leo Rover simulation](https://github.com/Alexandre-Frantz/leo_simulator-ros2). Mapping is possible using:

1. 3D LiDAR only
2. Fused 3D LiDAR and RGB-D camera. The 3D LiDAR cloud is colored using RGB values from the RGB-D camera

The sensors used are the following:

1. 3D LiDAR -> Livox MID 360
2. RGB-D Camera -> Realsense D455 RGB-D Camera

This package also provided an autonomous navigation option which can be used during and after SLAM. This feature is based on Nav2.

# Installation Instructions

The repository is based on ROS2 Humble, therefore please make sure you install this distro first, using this [link](https://docs.ros.org/en/humble/index.html)

## Install Required ROS2 Packages

1. **RTABMAP**

```bash
sudo apt install ros-humble-rtabmap-ros  
```

2. **Nav2**
Please follow the instructions in this [link](https://docs.nav2.org/getting_started/index.html)

## Clone and setup ROS2 workspace

1. Clone the repository into your ROS2 workspace (or create a new one)

```bash
cd <your_ros2_ws>/src/
git clone https://github.com/Alexandre-Frantz/rtabmap_livox_realsense.git
```

2. Build and source your workspace

```bash
cd <your_ros2_ws>
colcon build --symlink-install
source install/setup.bash
```

# Environment Mapping

With this package you can map using either the LiDAR or both LiDAR and RGB-D camera. Depending on your needs, the former produces a 3D point cloud with ground/obstacle segmentation, whereas the latter produces a colored point cloud.

## Mapping with LiDAR only

To launch RTAB-MAP with the Livox MID 360 LiDAR and map an environment, execute the following commands

```bash
ros2 launch rtabmap_livox_realsense rtabmap_livox.launch.py # Without namespace
ros2 launch rtabmap_livox_realsense rtabmap_livox.launch.py robot_namespace:=<robot_ns> # With namespace
```

> Note: RTABMAP saves a database file (rtabmap.db) under `.ros`. This file can be used for a future mapping session or to localize your robot in a previously mapped environment.

## Mapping with LiDAR and RGB-D camera

To launch RTAB-MAP with the Livox MID 360 LiDAR  and Realsense D455 and map an environment, execute the following commands

```bash
ros2 launch rtabmap_livox_realsense rtabmap_livox_rgbd_lidar_projection.launch.py # without namespace
ros2 launch rtabmap_livox_realsense rtabmap_livox_rgbd_lidar_projection.launch.py robot_namespace:=<robot_ns> # with namespace
```

> Note: RTABMAP saves a database file (rtabmap.db) under `.ros`. This file can be used for a future mapping session or to localize your robot in a previously mapped environment.

## Localizing your robot after a mapping session

You can also localize your robot after having mapped an environment. Depending on what type of mapping you did you may launch the following commands

```bash
ros2 launch rtabmap_livox_realsense rtabmap_livox_localization.launch.py 
ros2 launch rtabmap_livox_realsense rtabmap_livox_rgbd_lidar_projection_localization.launch.py 
```

You may also do this with namespaces if you have done your mapping with namespaces as well.

# Autonomous Navigation

To launch Nav2 execute the following:

```bash
ros2 launch rtabmap_livox_realsense custom_nav2_launch.py
```

# Contact information
Should you encounter any issues, require assistance or want to reach out please contact me:

Alexandre Frantz
alexandre.frantz@uni.lu
