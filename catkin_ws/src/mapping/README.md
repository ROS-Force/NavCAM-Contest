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

## Performance 

The tests were measured with the pre-recorded ``Test4_*.bag`` files at 30 FPS. The table shows the average Rate of odometry and map update to each testbench.


| Testbench                                | Odometry Rate (Hz) | Map Update Rate (Hz) |
| :-------------------------------------   | :----------------: | :------------------: |
| CPU: Ryzen 3700x, RAM:16GB, GPU: RTX3070 | 30.0               | 14.3                 |
| Jetson Xavier Nx (8GB)                   | 8.77               | 7.65                 |           
<!--| i5-7440HQ CPU @ 2.80GHz, RAM:8GB         | 2.39   | 3.89   |  26.67      | 2.60              |  1.65          |-->
