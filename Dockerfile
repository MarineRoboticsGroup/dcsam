FROM ubuntu:20.04

# Get dependencies:
RUN apt-get -y update && apt-get -y install cmake lsb-release curl

# Correctly install tzdata (which normally has interactive prompts) for Docker image
RUN DEBIAN_FRONTEND="noninteractive" TZ="America/New_York" apt-get -y install tzdata

# Update and install remaining package dependencies
RUN apt-get -y update && apt-get -y install sudo openssh-client \
  software-properties-common libgl1-mesa-dev libglew-dev libwayland-dev \
  libxkbcommon-dev wayland-protocols git

# Build and install GTSAM
RUN git clone https://github.com/borglab/gtsam.git && \
	mkdir gtsam/build && cd gtsam/build && \
  cmake .. && make -j8 install

# libgtest needed for DCSAM
RUN apt-get -y update && apt-get -y install libgtest-dev
RUN cd /usr/src/googletest && mkdir build && cd build && \
      cmake .. && make && cp googlemock/*.a googlemock/gtest/*.a /usr/lib

# User setup
ARG USER_ID
RUN adduser --uid $USER_ID mrg --disabled-password --gecos="mrg"
RUN usermod -aG sudo mrg
RUN echo "mrg ALL=NOPASSWD: ALL" >> /etc/sudoers

# Switch users NOTE(kevin): is this necessary?
USER mrg

WORKDIR /home/mrg/ws/src
