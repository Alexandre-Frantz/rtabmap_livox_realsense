# rtabmap_livox_realsense
ROS 2 package to launch RTAB-Map with the Leo Rover simulation of SpaceR using a 3D LiDAR and an RGB-D camera

To install RTAB-Map on ROS 2 Humble, use the following command:

```bash
sudo apt install ros-humble-rtabmap-ros
```

To run RTAB-Map with the Livox MID 360 LiDAR and a Realsense D-series camera, or the simulated sensors in Gazebo, in combination with the Leo Rover, run the following command:

```bash
ros2 launch rtabmap_livox_realsense rtabmap_livox_rgbd_lidar.launch.py
```
