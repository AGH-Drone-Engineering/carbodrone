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
COPY carbodrone_dualtech/package.xml ./carbodrone_dualtech/package.xml

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src/carbodrone/carbodrone_dualtech \
      --ignore-src \
    && apt-get install -y nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"

COPY carbodrone_dualtech ./src/carbodrone/carbodrone_dualtech

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        carbodrone_dualtech \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS=$OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

RUN useradd -m -d /home/user user
USER user

# run launch file
CMD ["ros2", "launch", "carbodrone_dualtech", "main.launch.py"]
