# NavCam-Contest

Work developed by ROS-Force for ActiveSpace's NavCAM contest.

## Overview

### Enviroment

This package was developed and tested in Ubuntu 18.04 and ROS melodic. It may not work in other enviroments.

- ##### Ubuntu 18.04
- ##### ROS Melodic
- ##### RTX 3070
- ##### python 2.7

### Camera

This package was tested using a [intel RealSense D435i camera](https://www.intelrealsense.com/depth-camera-d435i/).
The following table was made to evaluate the accuracy of the camera in good lighting conditions against an opaque material.

| Real Value (m)    | Mean (m)    | Standart Deviation (m) |
| ------------------|------------:|-----------------------:|
| 0.2               | 0.181       | 0.00048                |
| 1.0               | 1.038       | 0.0068                 |
| 3.0               | 3.153       | 0.078                  |
| 5.0               | 5.322       | 0.231                  |
| 7.0               | 7.707       | 0.723                  |
| 9.0               | 10.018      | 0.732                  |
| 10.0              | 11.453      | 1.22                   |

## Installation

This project is dependent on the following packages:

- ##### OpenCV
- ##### GSL
- ##### PCL

To use the funcionalities of CUDA you need to compile OpenCV from source, follow this tut.

TODO: adicionar os comandos para instalar as bibliotecas requeridas!

## [Object Detection](https://github.com/ROS-Force/NavCAM-Contest/tree/main/catkin_ws/src/object_detection)


