ARG BASE_IMAGE=ros:noetic-ros-base-l4t-r32.5.1
FROM ${BASE_IMAGE}
LABEL maintainer "José Faria <jncfaria@gmail.com>"

ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

#Add librealsense keys
RUN apt-get update && \
    apt-get install -y software-properties-common
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main"

# Add jetson keys
RUN sudo apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc
RUN add-apt-repository "deb https://repo.download.nvidia.com/jetson/common r32.5 main" 
RUN add-apt-repository "deb https://repo.download.nvidia.com/jetson/t194 r32.5 main"
RUN apt-get update && apt-get -y dist-upgrade

# Install toolchain & other tools
RUN apt-get -y install \
    build-essential \
    automake \
    gettext \
    curl \
    git \
    graphviz \
    unzip \
    wget \
    gcc \
    net-tools \
    nano \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    libpcl-dev \
    libyaml-cpp-dev \
    libgeographic-dev \
    libbullet-dev \
    libpcap-dev \
    libeigen3-dev \ 
    libomp-dev \
    libopencv \
    libopencv-dev \
    libopencv-python

# Install TF dependencies 
RUN apt-get -y install \
    libhdf5-serial-dev \
    hdf5-tools \ 
    libhdf5-dev \ 
    zlib1g-dev \ 
    zip \
    libjpeg8-dev \ 
    liblapack-dev \ 
    libblas-dev \ 
    gfortran

# Install some python packages
RUN apt-get -y install \
    python3 \
    python3-pip \
    python3-serial \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep

# update pip


# install more TF dependencies
#RUN pip3 install -U \ 
#    future==0.18.2 \
#    mock==3.0.5  \
#    h5py==2.10.0 \
#    keras_preprocessing==1.1.1 \
#    keras_applications==1.0.8 \
#    gast==0.2.2 \
#    futures \
#    protobuf \
#    pybind11

RUN pip3 install --upgrade pip

#Install more PIP packages (for SORT algorithm and coloring for bounding boxes)
RUN pip3 install \
    colorutils \
    pathfinding \
    lap \
    filterpy

# install tensorflow 
#RUN pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v45 tensorflow

# Clean-up
RUN apt-get clean


# Install catkin tools manually
WORKDIR /root/tools
RUN git clone https://github.com/catkin/catkin_tools.git
WORKDIR /root/tools/catkin_tools
RUN pip3 install -r requirements.txt
RUN python3 setup.py install --record install_manifest.txt

#===========SETUP SSH============================

# Authorize SSH Host
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh && \
    ssh-keyscan github.com > /root/.ssh/known_hosts

ARG ssh_prv_key
ARG ssh_pub_key

RUN echo $ssh_prv_key
RUN echo $ssh_pub_key

# Add the keys and set permissions 
RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
    chmod 600 /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa.pub

#Install NavCAM repo
WORKDIR /root
RUN git clone -b noetic --single-branch --recursive git@github.com:jncfa/NavCAM-Contest.git
# hotfixes for certain packages
RUN sed -i 's/python37/python/' /root/NavCAM-Contest/catkin_ws/src/packagedeps/vision_opencv/cv_bridge/CMakeLists.txt

#==========COMPILE DEPENDENCIES===============
#RUN mkdir /root/NavCAM-Contest/depends/opencv/build
#WORKDIR /root/NavCAM-Contest/depends/opencv/build
#RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
#-D CMAKE_INSTALL_PREFIX=/usr/local \
#-D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
#-D INSTALL_PYTHON_EXAMPLES=ON \
#-D INSTALL_C_EXAMPLES=ON \
#-D OPENCV_ENABLE_NONFREE=ON \
#-D WITH_CUDA=ON \
#-D WITH_CUDNN=ON \
#-D WITH_EIGEN=ON \
#-D OPENCV_DNN_CUDA=ON \
#-D ENABLE_FAST_MATH=1 \
#-D CUDA_FAST_MATH=1 \
#-D CUDA_ARCH_BIN=7.2 \
#-D CUDA_ARCH_PTX= \
#-D WITH_CUBLAS=1 \
#-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
#-D HAVE_opencv_python2=OFF \
#-D HAVE_opencv_python3=ON \
#-D BUILD_opencv_python2=OFF \
#-D BUILD_opencv_python3=ON \
#-D OPENCV_GENERATE_PKGCONFIG=ON \
#-D BUILD_EXAMPLES=OFF ..
#RUN make -j`nproc`
#RUN make install

RUN mkdir /root/NavCAM-Contest/depends/g2o/build
WORKDIR /root/NavCAM-Contest/depends/g2o/build
RUN cmake ..
RUN make -j`nproc`
RUN make install

RUN mkdir /root/NavCAM-Contest/depends/libnabo/build
WORKDIR /root/NavCAM-Contest/depends/libnabo/build
RUN cmake -DLIBNABO_BUILD_PYTHON=OFF ..
RUN make -j`nproc`
RUN make install

RUN mkdir /root/NavCAM-Contest/depends/libpointmatcher/build
WORKDIR /root/NavCAM-Contest/depends/libpointmatcher/build
RUN cmake ..
RUN make -j`nproc`
RUN make install

RUN mkdir /root/NavCAM-Contest/depends/gtsam/build
WORKDIR /root/NavCAM-Contest/depends/gtsam/build
RUN cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
RUN make -j`nproc`
RUN make install

RUN mkdir /root/NavCAM-Contest/depends/octomap/build
WORKDIR /root/NavCAM-Contest/depends/octomap/build
RUN cmake ..
RUN make -j`nproc`
RUN make install

#RUN mkdir /root/NavCAM-Contest/depends/rtabmap/build
WORKDIR /root/NavCAM-Contest/depends/rtabmap/build
RUN cmake -DWITH_PYTHON=ON -DBUILD_EXAMPLES=OFF ..
RUN make -j`nproc`
RUN make install

#==================COMPILE THE REPO==============================
WORKDIR /root/NavCAM-Contest/catkin_ws
RUN catkin init
RUN catkin config --profile Release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN catkin profile set Release
RUN catkin config --install --merge-devel --blacklist rviz_imu_plugin --extend $ROS_ROOT

# build orocos first and source it
RUN catkin build orocos_kdl -j`nproc`

# build the rest 
RUN catkin build -j`nproc`
RUN echo "source /root/NavCAM-Contest/catkin_ws/devel_release/setup.bash --extend" >> /root/.bashrc

EXPOSE 11311
CMD ["/bin/bash"]
