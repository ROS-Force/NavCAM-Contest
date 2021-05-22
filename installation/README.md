# Install dependencies

## ROS Noetic

The best way to install ROS is to follow [this](http://wiki.ros.org/noetic/Installation/Ubuntu) tutorial. Choose the Desktop-Full Install to install packages like rviz to help visualize the results.

We need a few more ROS related packages, install them with the following command:
```bash
sudo apt install ros-noetic-octomap ros-noetic-ddynamic-reconfigure
```

### Optional

After the installation you can edit your ~/.bashrc file to avoid having to source all the time. Add the following lines to the end of your ~/.bashrc file:

```bash
source /opt/ros/noetic/setup.bash
source ~/NavCAM-Contest/catkin_ws/devel/setup.bash --extend
```

### Intel® RealSense™ D435i 

Follow [this](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) tutorial to install [**Intel® RealSense™ SDK 2.0**](https://github.com/IntelRealSense/librealsense).

## Other Python packages

To install the packages, run:

```bash
pip3 install tensorflow catkin_tools colorutils pathfinding
```

## CUDA

The best way to install CUDA is to follow [this](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) tutorial. This package was tested with version 11.2 of CUDA.

## cuDNN

In order to setup cuDNN in your system, there are several methods. Here, we will follow the package manager installation, fOr more information or a more detailed tutorial follow the [NVIDIA cuDNN Documentation guide](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html). 

First we need to add the NVIDIA CUDA repositories. 
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin 

sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get update
```

```bash
# for the latest cuDNN version (ensure this matches your CUDA version)
sudo apt-get install libcudnn8
sudo apt-get install libcudnn8-dev
```

```bash
# if you wish to install for a specific cuDNN version & CUDA version, then replace the values below
export cudnn_version=8.1.1.33-1
export cuda_version=11.2
sudo apt-get install libcudnn8=${cudnn_version}-1+${cuda_version}
sudo apt-get install libcudnn8-dev=${cudnn_version}-1+${cuda_version}
```

## Compile OpenCV from source

First step is to create a build folder:
```bash
cd NavCAM-Contest/depends/opencv
mkdir build
cd build
```

Then copy the following command and **change** the **CUDA_ARCH_BIN** value to the compute capability of your machine. If you don't know your compute capability look for your Graphic Card [here](https://developer.nvidia.com/cuda-gpus).

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

## RTABMAP

To properly take advantage of the GPU acceleration features provided by certain OpenCV packages on RTABMAP, we will need to compile RTAMAP from source to link to the our CUDA-enabled version. Before we compile RTABMAP however, we will also need to compile some additional packages:

```bash
mkdir /root/NavCAM-Contest/depends/g2o
cd /root/NavCAM-Contest/depends/g2o/build
cmake ..
make -j`nproc`
make install

mkdir /root/NavCAM-Contest/depends/libnabo/build
cd /root/NavCAM-Contest/depends/libnabo/build
cmake -DLIBNABO_BUILD_PYTHON=OFF ..
make -j`nproc`
make install

mkdir NavCAM-Contest/depends/libpointmatcher/build
cd /root/NavCAM-Contest/depends/libpointmatcher/build
cmake ..
make -j`nproc`
make install

mkdir NavCAM-Contest/depends/gtsam/build
cd /root/NavCAM-Contest/depends/gtsam/build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j`nproc`
make install

mkdir NavCAM-Contest/depends/octomap/build
cd /root/NavCAM-Contest/depends/octomap/build
cmake ..
make -j`nproc`
make install
```

Finally, we build and install RTABMAP:

```bash
mkdir NavCAM-Contest/depends/rtabmap/build
cd /root/NavCAM-Contest/depends/rtabmap/build
cmake -DWITH_PYTHON=ON -DBUILD_EXAMPLES=OFF ..
make -j`nproc` 
make install
```

