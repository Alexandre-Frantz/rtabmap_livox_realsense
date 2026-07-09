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

To launch RTAB-MAP with the Livox MID 360 LiDAR and map an environment, execute either the following commands

```bash
ros2 launch rtabmap_livox_realsense rtabmap_livox.launch.py # Without namespace
ros2 launch rtabmap_livox_realsense rtabmap_livox.launch.py robot_namespace:=<robot_ns> # With namespace
```

> Note: RTABMAP saves a database file (rtabmap.db) under `.ros`. This file can be used for a future mapping session or to localize your robot in a previously mapped environment.

## Mapping with LiDAR and RGB-D camera

To launch RTAB-MAP with the Livox MID 360 LiDAR  and Realsense D455 and map an environment, execute either the following commands

```bash
ros2 launch rtabmap_livox_realsense rtabmap_livox_rgbd_lidar_projection.launch.py # without namespace
ros2 launch rtabmap_livox_realsense rtabmap_livox_rgbd_lidar_projection.launch.py robot_namespace:=<robot_ns> # with namespace
```

> Note: RTABMAP saves a database file (rtabmap.db) under `.ros`. This file can be used for a future mapping session or to localize your robot in a previously mapped environment.

## Localizing your robot after a mapping session

You can also localize your robot after having mapped an environment. Depending on what type of mapping you did you may launch either of the following commands

```bash
ros2 launch rtabmap_livox_realsense rtabmap_livox_localization.launch.py # mapping done with LiDAR only
ros2 launch rtabmap_livox_realsense rtabmap_livox_rgbd_lidar_projection_localization.launch.py  # mapping done with LiDAR and RGB-D Camera
```

You may also do this with namespaces if you have done your mapping with namespaces as well.

# Autonomous Navigation

To launch Nav2 execute the following:

```bash
ros2 launch rtabmap_livox_realsense custom_nav2_launch.py
```

## Namespaced Nav2 (multi-robot)

For multi-robot operation each robot runs its own fully namespaced Nav2 stack using
`custom_nav2_namespaced_launch.py`. It launches the Nav2 nodes directly under a
`PushRosNamespace`, so all topics and TF frames are scoped to the robot:

```bash
ros2 launch rtabmap_livox_realsense custom_nav2_namespaced_launch.py robot_namespace:=leo04
ros2 launch rtabmap_livox_realsense custom_nav2_namespaced_launch.py robot_namespace:=leo05
```

Launch arguments:
  * `robot_namespace`: robot namespace, must match the RTAB-Map launch (default: `leo04`)
  * `use_sim_time`: use the Gazebo clock (default: `true`; set `false` on the real robot)
  * `params_file`: Nav2 params YAML (default: `config_files/nav2_params_repo.yaml`)
  * `autostart`: auto-activate the Nav2 lifecycle nodes (default: `true`)

Notes:
  * The params file uses `leo04` as a namespace placeholder; it is replaced with
    `robot_namespace` at launch time, so a single YAML works for every robot.
  * RTAB-Map publishes namespaced TF frames (e.g. `leo04/odom`) to the global `/tf`,
    so Nav2 subscribes to the global `/tf` (no `/tf` remapping).
  * To send goals from RViz, set the **2D Goal Pose** tool topic to `/<robot_namespace>/goal_pose`
    (RViz → Tool Properties), otherwise the default `/goal_pose` will not reach `bt_navigator`.

# Contact information
Should you encounter any issues, require assistance or want to reach out please contact me:

Alexandre Frantz

alexandre.frantz@uni.lu
