# CUDA 11.7, TensorRT 8.4.1
FROM nvcr.io/nvidia/tensorrt:22.07-py3

ARG DEBIAN_FRONTEND=noninteractive

# Basic
RUN apt-get update -y && \
    apt-get install -y sudo libeigen3-dev libboost-all-dev libssl-dev libyaml-cpp-dev wget build-essential cmake gdb libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev libc++-dev libglew-dev libeigen3-dev cmake g++ ninja-build cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

# OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip && \
    unzip opencv.zip && \
    cd opencv-4.4.0  && \
    mkdir build && cd build && \
    cmake .. && make -j4 && make install

# Pangolin
RUN wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip &&\
    unzip v0.6.zip &&\
    rm -rf v0.6.zip &&\
    cd Pangolin-0.6 && mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    cd ../../

# SuperSLAM
RUN git clone https://github.com/prgrms-ad-devcourse/ad-5-final-project-team2.git &&\
    cd ad-5-final-project-team2 &&\
    sh ./build.sh