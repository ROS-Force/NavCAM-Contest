# Mapping

To perform simultaneous mapping and localization (SLAM), we use the following packages:

- **OctoMap** - 3D occupancy grid mapping approach;
- **RTAB-Map** - Visual odometry (PnP, using GFTT and ORB features) and loop closure, integrating OctoMap mapping;
- **robot_localization** - State estimation node using Unscented Kalman Filter (UFK) to fuse Visual Odometry with IMU data;

This package contains the launch files to run the packages automatically. To start the SLAM process, use the following [`slam-navigation.launch`](launch/slam-navigation.launch). You can use the parameters `with_camera` to enable the camera automatically, and `with_path_finding` to start the path finding node, like this:

```bash
  roslaunch mapping slam-navigation.launch with_camera:=true with_path_finding:=true
```

## Demos

![](img/2d_mapping.gif)
![](img/3d_mapping.gif)
