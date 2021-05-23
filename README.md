# NavCAM-Contest

Work developed by ROS-Force for ActiveSpace's NavCAM contest.

## Overview

### Enviroment

This package was developed and tested with the following software:

- **Ubuntu 20.04**
- **ROS Noetic**
- **Python 3**
- **CUDA v11.2**
- **cuDNN v8.1.1**
- **OpenCV v4.5.1**
- **Tensorflow v2.4.1**

To compile this package for the **Nvidia Jetson Xavier NX** please go to [this fork]().

### Camera

This package was tested using a [Intel RealSense D435i camera](https://www.intelrealsense.com/depth-camera-d435i/).
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

## Setting up the packages

The first step is to clone the repository:

```bash
cd ~
git clone --recursive git@github.com:ROS-Force/NavCAM-Contest.git
```
However, before we build and run our packages, we will need to install some dependencies first.
____

### Dependencies

#### ROS Noetic

The best way to install ROS is to follow [this](http://wiki.ros.org/noetic/Installation/Ubuntu) tutorial. Follow the Desktop-Full installation to install packages like RVIZ to help visualize the results.

We need a few more ROS related packages, install them with the following command:

```bash
sudo apt install ros-noetic-octomap ros-noetic-ddynamic-reconfigure
```

#### Intel® RealSense™ D435i 

Follow [this](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) tutorial to install [**Intel® RealSense™ SDK 2.0**](https://github.com/IntelRealSense/librealsense).

#### Other Python packages

To install the packages, run:

```bash
pip3 install tensorflow catkin_tools colorutils pathfinding
```

#### CUDA

The best way to install CUDA is to follow [this](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) tutorial. This package was tested with version 11.2 of CUDA.

#### cuDNN

In order to setup cuDNN in your system, there are several methods. Here, we will follow the package manager installation, fOr more information or a more detailed tutorial follow the [NVIDIA cuDNN Documentation guide](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html). 

First we add the apt repositories and install the needed packages:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin 

sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt update
```

Copy this commands to install the latest cuDNN version:
```bash
# for the latest cuDNN version (ensure this matches your CUDA version)
sudo apt install libcudnn8
sudo apt install libcudnn8-dev
```
**OR** use these commands to install a specific version of cuDNN:

```bash
# if you wish to install for a specific cuDNN version & CUDA version, then replace the values below
export cudnn_version=8.1.1.33-1
export cuda_version=11.2
sudo apt install libcudnn8=${cudnn_version}-1+${cuda_version}
sudo apt install libcudnn8-dev=${cudnn_version}-1+${cuda_version}
```

#### Compile OpenCV from source

To make use of GPU acceleration, we need to compile OpenCV from source. First step is to create a build folder:

```bash
cd NavCAM-Contest/depends/opencv
mkdir build
cd build
```

Then copy the following command and **change** the **CUDA_ARCH_BIN** value to the compute capability of your machine. If you don't know your compute capability look for your Graphic Card [here](https://developer.nvidia.com/cuda-gpus):

```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D BUILD_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D WITH_EIGEN=ON \
-D OPENCV_DNN_CUDA=ON \
-D ENABLE_FAST_MATH=1 \
-D CUDA_FAST_MATH=1 \
-D CUDA_ARCH_BIN=6.1 \
-D WITH_CUBLAS=1 \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
-D HAVE_opencv_python2=ON \
-D HAVE_opencv_python3=ON \
-D BUILD_opencv_python2=ON \
-D BUILD_opencv_python3=ON \
-D BUILD_EXAMPLES=ON ..
```
Now compile:

```bash
# you can change the amount of make jobs with the -j argument to speed up the compilation process
make -j`nproc`
sudo make install
```

and you should be done!

#### RTAB-Map

[**RTAB-Map** (Real-Time Appearance-Based Mapping)](http://introlab.github.io/rtabmap/) is a RGB-D SLAM approach based on a global loop closure detector with real-time constraints. It can be used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation. 

In order to maximize performance on RTAB-Map, we need to take advantage of the GPU acceleration features provided by certain OpenCV packages used by RTAB-Map, as well as faster mapping by mixing with another mapping algorithm called [Octomap](https://octomap.github.io/). Which means, we will need to compile RTAB-Map from source to link to the our CUDA-enabled version and to the Octomap libraries. Before we compile RTAB-Map however, we will also need to compile some additional packages:

```bash
# building g2o for graph optimization
mkdir /root/NavCAM-Contest/depends/g2o
cd /root/NavCAM-Contest/depends/g2o/build
cmake ..
make -j`nproc`
sudo make install

# building libnabo for libpointmatcher
mkdir /root/NavCAM-Contest/depends/libnabo/build
cd /root/NavCAM-Contest/depends/libnabo/build
cmake -DLIBNABO_BUILD_PYTHON=OFF ..
make -j`nproc`
make install

# building libpointmatcher for better ICP algorithm
mkdir NavCAM-Contest/depends/libpointmatcher/build
cd /root/NavCAM-Contest/depends/libpointmatcher/build
cmake ..
make -j`nproc`
sudo make install

# building GTSAM for graph optmization (if you installed g2o, you don't have to install this one)
mkdir NavCAM-Contest/depends/gtsam/build
cd /root/NavCAM-Contest/depends/gtsam/build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j`nproc`
sudo make install

# building Octomap for better & faster mapping
mkdir NavCAM-Contest/depends/octomap/build
cd /root/NavCAM-Contest/depends/octomap/build
cmake ..
make -j`nproc`
sudo make install
```

Finally, we build and install RTAB-Map:

```bash
mkdir NavCAM-Contest/depends/rtabmap/build
cd /root/NavCAM-Contest/depends/rtabmap/build
cmake -DWITH_PYTHON=ON -DBUILD_EXAMPLES=OFF ..
make -j`nproc` 
sudo make install
```
___

After installing all the required dependencies, configure and compile the catkin workspace:

```bash
cd ~/NavCAM-Contest/catkin_ws

catkin init
catkin config --profile Release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin profile set Release
catkin config --install --merge-devel --extend /opt/ros/$ROS_DISTRO
catkin build -j`nproc` 
source ~/NavCAM-Contest/catkin_ws/devel_release/setup.bash
```

After the installation, edit your `~/.bashrc` file to avoid having to source all the time:

```bash
echo "source ~/NavCAM-Contest/catkin_ws/devel_release/setup.bash --extend" >> ~/.bashrc
```

# Sub-Packages

## [Object Detection](/catkin_ws/src/object_detection)

## [Mapping](/catkin_ws/src/mapping)

## [Path Finding](/catkin_ws/src/path_finding)
