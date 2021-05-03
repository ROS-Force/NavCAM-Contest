# NavCam-Contest

Work developed by ROS-Force for ActiveSpace's NavCAM contest.

## Overview

### Enviroment

This package was developed and tested in Ubuntu 18.04 and ROS melodic. It may not work in other enviroments.

- **Ubuntu 20.04**
- **ROS Noetic**
- **python 3**
- **CUDA v11.2**
- **cuDNN v8.1.1**

### Camera

This package was tested using a [intel RealSense D435i camera](https://www.intelrealsense.com/depth-camera-d435i/).
The following table was made to evaluate the accuracy of the camera in good lighting conditions against an opaque material.

| Real Value (m) | Mean (m) | Standart Deviation (m) |
| -------------- | -------: | ---------------------: |
| 0.2            |    0.181 |                0.00048 |
| 1.0            |    1.038 |                 0.0068 |
| 3.0            |    3.153 |                  0.078 |
| 5.0            |    5.322 |                  0.231 |
| 7.0            |    7.707 |                  0.723 |
| 9.0            |   10.018 |                  0.732 |
| 10.0           |   11.453 |                   1.22 |

## Dependencies

This project is dependent on the following packages:

- **OpenCV**
- **ROS**
- **TensorFlow >= 2.0**
- **CUDA - v11.2**


## Install this package

The first step is to clone the repository. Go to your $HOME folder and clone it.

    cd ~
    git clone git@github.com:ROS-Force/NavCAM-Contest.git

Then update all the submodules

    cd ~/NavCAM-Contest
    git submodules update --init --recursive

Now we advised to follow [our guide](https://github.com/ROS-Force/NavCAM-Contest/tree/main/installation) on how to build the dependencies.

After installing all the required dependencies compile the catkin workspace:
    
    cd ~/NavCAM-Contest/catkin_ws
    catkin_make
or

    cd ~/NavCAM-Contest/catkin_ws
    catkin build

## [Object Detection](https://github.com/ROS-Force/NavCAM-Contest/tree/main/catkin_ws/src/object_detection)
