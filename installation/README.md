# Install dependencies

## ROS Melodic

The best way to install ROS is to follow [this](http://wiki.ros.org/melodic/Installation/Ubuntu) tutorial. Choose the Desktop-Full Install to install packages like rviz to help vizualize the results.

We need a few more ROS related packages, install them with the following command:

    sudo apt install ros-noetic-octomap ros-noetic-ddynamic-reconfigure

## Intel Camera Python Wrapper

Information is available at the [Python Wrapper](https://github.com/IntelRealSense/librealsense/tree/development/wrappers/python)

To install the package, run:

    pip3 install pyrealsense2

## CUDA

The best way to install CUDA is to follow [this](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) tutorial. This package was teste with version 11.2 of CUDA.

## cuDNN



In order to download cuDNN, ensure you are registered for the [NVIDIA Developer Program](https://developer.nvidia.com/developer-program). Then go to [NVIDIA cuDNN home page](https://developer.nvidia.com/cudnn) and click to download.

We advise to install the Debian packages, therefore download the 3 \*.deb files for your Ubuntu version.

    cd ~/Downloads
    sudo dpkg -i libcudnn*.deb

and cuDNN should be ready to go. For more information or a more detailed tutorial follow the [NVIDIA cuDNN Documentation guide](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html).

## Compile OpenCV from source

First step is to create a build folder:

    cd NavCAM-Contest/depends/opencv
    mkdir build
    cd build

Then copy the following command and **change** the **CUDA_ARCH_BIN** value to the compute capability of your machine. If you don't know your compute capability look for your Graphic Card [here](https://developer.nvidia.com/cuda-gpus).

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

and you should be done

## TensorFlow

# Other packages
