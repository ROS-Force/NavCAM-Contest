# NavCam-Contest

Work developed by ROS-Force for ActiveSpace's NavCAM contest.

## Overview

### Enviroment

This package was developed and tested in Ubuntu 18.04 and ROS melodic. It may not work in other enviroments.

- **Ubuntu 18.04**
- **ROS Melodic**
- **python 2.7**
- **CUDA v10.2**
- **cuDNN v7.6.5**

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
- **GSL**
- **ROS** (Only melodic is assured)
- **OpenPose**
- **TensorFlow < 2.4.0**
- **CUDA - v10.0**
- **cuDNN - v7.6.5**

As of this moment (04/2021) Caffe framework only suport cuDNN <= v7.6.5. So it is **NOT** advised to install the latest CUDA and cuDNN version.

## Install this package

The first step is to clone the repository. Go to your $HOME folder and clone it.

    cd ~
    git clone git@github.com:ROS-Force/NavCAM-Contest.git

Then update all the submodules

    cd ~/NavCAM-Contest
    git submodules update --innit --recursive

Now we advised to follow [our guide](https://github.com/ROS-Force/NavCAM-Contest/tree/main/installation) on how to build the dependencies.

## [Object Detection](https://github.com/ROS-Force/NavCAM-Contest/tree/main/catkin_ws/src/object_detection)
