# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM osrf/ros:jazzy-desktop-full

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
    
# Install PCL & Eigen & essential libraries
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        cppcheck \
        gdb \
        git \
        sudo \
        vim \
        wget \
        tmux \
        curl \
        less \
        htop \
        libsm6 libxext6 libgl1 libglx-mesa0 libxrender-dev \
        curl \
 && apt-get clean

RUN export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean
 
RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip git python3-pip
RUN apt-get install -y ros-jazzy-tf2 ros-jazzy-cv-bridge ros-jazzy-pcl-conversions ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 ros-jazzy-image-transport ros-jazzy-image-transport-plugins ros-jazzy-pcl-ros

RUN apt-get install -y libceres-dev 
    

RUN apt-get install -y python3-tk python3-pip

# Python 3.12 on Ubuntu 24.04 marks the system interpreter as externally
# managed (PEP 668), so pip needs --break-system-packages inside the image.
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN pip3 install --upgrade pip
RUN pip3 install --no-cache-dir numpy scipy matplotlib ipython opencv-python 
RUN pip3 install --no-cache-dir pandas visdom scikit-image scikit-learn numba Pillow
RUN pip3 install --no-cache-dir pyyaml pycryptodome Cython

# python3-rosinstall and python3-wstool are ROS 1 tooling and are gone from
# Ubuntu 24.04; catkin_tools / gym were unused and do not build on Python 3.12.
RUN apt-get -y install python3-rosdep python3-rosinstall-generator build-essential
# RUN sudo rosdep init
RUN rosdep update
# Deliberately not the ros-jazzy-grid-map metapackage: it would also install
# grid-map-pcl and grid-map-filters-rsl, which this repo builds from source
# with local extensions.
RUN apt-get -y install ros-jazzy-grid-map-core ros-jazzy-grid-map-msgs \
    ros-jazzy-grid-map-ros ros-jazzy-grid-map-cv ros-jazzy-grid-map-cmake-helpers

# Install livox SDK
WORKDIR /root/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /root/Livox-SDK2
RUN mkdir build
WORKDIR /root/Livox-SDK2/build
RUN cmake .. && make -j2 && make install

WORKDIR /root/
RUN git clone http://github.com/strasdat/Sophus.git \
    && cd Sophus && git checkout 97e7161 \
    && mkdir build && cd build && cmake .. -DBUILD_TESTS=OFF \
    && make -j8 && sudo make install
WORKDIR /root/
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && git checkout 4abef92 &&\
    mkdir build && \
    cd build && \
    cmake  \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
           .. && \
    make -j6 && \
    sudo make install


RUN /bin/bash -c '. /opt/ros/jazzy/setup.bash;'

RUN pip install rerun-sdk
# Ubuntu 24.04 ships numpy 2.x for Python 3.12; numpy 1.24 has no cp312 wheel.
RUN pip install "numpy>=2.0,<3" "matplotlib>=3.8"
RUN pip install --upgrade packaging
RUN pip3 install torch --ignore-installed


WORKDIR /root/ros2_ws

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]

























