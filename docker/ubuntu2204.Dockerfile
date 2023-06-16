# Dockerfile for registry.freedesktop.org/mateosss/basalt/ubuntu/22.04
# This image is used to build Basalt .deb package for Ubuntu 22.04

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update
RUN apt-get install -y gcc
RUN apt-get install -y g++
RUN apt-get install -y cmake
RUN apt-get install -y git
RUN apt-get install -y libtbb-dev
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y libglew-dev
RUN apt-get install -y ccache
RUN apt-get install -y libjpeg-dev
RUN apt-get install -y libpng-dev
RUN apt-get install -y liblz4-dev
RUN apt-get install -y libbz2-dev
RUN apt-get install -y libboost-regex-dev
RUN apt-get install -y libboost-filesystem-dev
RUN apt-get install -y libboost-date-time-dev
RUN apt-get install -y libboost-program-options-dev
RUN apt-get install -y libgtest-dev
RUN apt-get install -y libopencv-dev
RUN apt-get install -y libfmt-dev
RUN apt-get clean
