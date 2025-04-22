ARG FROM_IMAGE=docker.io/ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

WORKDIR $OVERLAY_WS/src/carbodrone
COPY carbodrone_picamera/package.xml ./carbodrone_picamera/package.xml

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# picamera2

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
ENV PYTHONPATH=/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages:/usr/local/lib/x86_64-linux-gnu/python3.10/site-packages

# !picamera2

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src/carbodrone/carbodrone_picamera \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"

COPY carbodrone_picamera ./src/carbodrone/carbodrone_picamera

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        carbodrone_picamera \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS=$OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

USER root
CMD ["ros2", "run", "carbodrone_picamera", "camera_node.py"]
