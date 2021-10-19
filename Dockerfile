FROM ubuntu:20.04

# Correctly install tzdata (which normally has interactive prompts) for Docker image
RUN DEBIAN_FRONTEND="noninteractive" TZ="America/New_York" apt-get -y update && apt-get -y install tzdata

# Get dependencies:
RUN apt-get -y update && apt-get -y install cmake lsb-release curl build-essential gnutls-bin

# Update and install remaining package dependencies
RUN apt-get -y update && apt-get -y install sudo openssh-client \
  software-properties-common libgl1-mesa-dev libglew-dev libwayland-dev \
  libxkbcommon-dev wayland-protocols git

# GCC/G++9 needed
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get -y update && apt-get -y install gcc-9 g++-9

# Ensure we use the gcc-9 and g++-9
RUN export CC=/usr/bin/gcc-9
RUN export CXX=/usr/bin/g++-9

# libboost required by GTSAM
RUN apt-get -y update && apt-get -y install libboost-all-dev libtbb-dev

# Build and install GTSAM
RUN git clone https://github.com/borglab/gtsam.git && \
	mkdir gtsam/build && cd gtsam/build && \
  cmake .. && make -j8 install

# libgtest needed for DCSAM
RUN apt-get -y update && apt-get -y install libprotobuf-dev protobuf-compiler libgtest-dev

RUN cd /usr/src/gtest && mkdir build && cd build && cmake .. -DBUILD_SHARED_LIBS=ON && make && ls lib && cp /usr/src/gtest/build/lib/* /usr/include/

# Eigen3.3 needed for DCSAM
RUN apt-get -y update && apt-get -y install libeigen3-dev

# RUN cd /usr/src/gmock && mkdir build && cd build && \
#       cmake .. && make && sp /usr/src/gmock/*.a /usr/lib/

# User setup
#ARG USER_ID
#RUN adduser --uid $USER_ID mrg --disabled-password --gecos="mrg"
#RUN usermod -aG sudo mrg
#RUN echo "mrg ALL=NOPASSWD: ALL" >> /etc/sudoers

# Switch users NOTE(kevin): is this necessary?
#USER mrg

#WORKDIR /home/mrg/ws/src
