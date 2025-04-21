FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
         python3-pip \
         git \
         python3-jinja2 \
         libboost-dev \
         libgnutls28-dev openssl libtiff-dev pybind11-dev \
         cmake ninja-build \
         python3-yaml python3-ply \
         libfmt-dev libdrm-dev libfmt-dev \
         libcap-dev \
     && apt-get clean \
     && rm -rf /var/cache/apt/archives/* \
     && rm -rf /var/lib/apt/lists/*

RUN pip3 install meson

WORKDIR /build
RUN git clone https://github.com/raspberrypi/libcamera.git
WORKDIR /build/libcamera
RUN meson setup build --buildtype=release -Dpipelines=rpi/pisp -Dipas=rpi/pisp -Dv4l2=false -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
RUN ninja -C build install
RUN ldconfig


WORKDIR /build
RUN git clone https://github.com/tomba/kmsxx.git
WORKDIR /build/kmsxx
RUN git reset --hard 6cf6e88715ac034f568603bce9a1b8f4a30c12ce
RUN git submodule update --init
RUN meson build -Dpykms=enabled
RUN ninja -C build install
RUN ldconfig

WORKDIR /build
RUN git clone https://github.com/raspberrypi/picamera2.git
WORKDIR /build/picamera2
RUN python3 setup.py install
RUN ldconfig
ENV PYTHONPATH=/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages
