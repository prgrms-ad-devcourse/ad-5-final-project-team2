# CUDA 11.7, TensorRT 8.4.1
FROM nvcr.io/nvidia/tensorrt:22.07-py3

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install sudo libeigen3-dev wget build-essential cmake && \
    # OpenCV
    apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y && \
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip && \
    unzip opencv.zip && \
    cd opencv-4.4.0  && \
    mkdir build && cd build && \
    cmake .. && make -j4 && make install

RUN cd ../ && \
    apt-get install libyaml-cpp-dev -y && \
    git clone https://github.com/yuefanhao/SuperPoint-SuperGlue-TensorRT.git && \
    cd SuperPoint-SuperGlue-TensorRT && mkdir build && cd build && cmake .. && make -j4

RUN apt-get install -y libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev libc++-dev libglew-dev libeigen3-dev cmake g++ ninja-build


# Pangolin
RUN wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip &&\
    unzip v0.6.zip &&\
    cd Pangolin-0.6 && mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    cd ../../

# ORB-SLAM3
RUN apt-get install -y libboost-all-dev libssl-dev
RUN wget https://github.com/UZ-SLAMLab/ORB_SLAM3/archive/refs/tags/v1.0-release.zip &&\
    unzip v1.0-release.zip &&\
    cd ORB_SLAM3-1.0-release &&\
    sh build.sh