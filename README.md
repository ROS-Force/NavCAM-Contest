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

As of this moment (04/2021) Caffe framework only suport cuDNN <= v7.6.5. So it is **NOT** advised to install the latest CUDA and cuDNN version. Please follow [this](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) tutorial to install CUDA.

## Install this package

The first step is to clone the repository. Go to your $HOME folder and clone it.

    cd ~
    git clone git@github.com:ROS-Force/NavCAM-Contest.git

Then update all the submodules

    cd ~/NavCAM-Contest
    git submodules update --innit --recursive

Now we advised to follow the guide on how to build the dependencies.

### cuDNN

In order to download cuDNN, ensure you are registered for the [NVIDIA Devveloper Program](https://developer.nvidia.com/developer-program). Then go to [NVIDIA cuDNN home page](https://developer.nvidia.com/cudnn) and click to download.

We advise to install the Debian packages, therefore download the 3 \*.deb files for your Ubuntu version (ensure the cuDNN version is <= 7.6.5). Then go to the folder that constains the downloaded packages, probably ~/Downloads.

    cd ~/Downloads
    sudo apt install libcudnn*.deb

and cuDNN should be ready to go. For more information or a more detailed tutorial follow the [NVIDIA cuDNN Documentation guide](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html).

### Compile OpenCV from source

First step is to create a build folder:

    cd NavCAM-Contest/depends/opencv
    mkdir build
    cd build

Then copy the following command and **change** the **CUDA_ARCH_BIN=6.1** to the compute capability of your machine. If you don't know your compute capability look for your Graphic Card [here](https://developer.nvidia.com/cuda-gpus).

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

Now compile

    make
    sudo make install

and you should be done!

### Compile OpenPose from source

## [Object Detection](https://github.com/ROS-Force/NavCAM-Contest/tree/main/catkin_ws/src/object_detection)
