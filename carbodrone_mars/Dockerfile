ARG FROM_IMAGE=ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
RUN echo "\
repositories: \n\
  px4_msgs: \n\
    type: git \n\
    url: https://github.com/PX4/px4_msgs.git \n\
  px4_ros_com: \n\
    type: git \n\
    url: https://github.com/PX4/px4_ros_com.git \n\
" > ../overlay.repos
RUN vcs import ./ < ../overlay.repos

WORKDIR $OVERLAY_WS/src/carbodrone/carbodrone_mars
COPY package.xml ./

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
        src/carbodrone/carbodrone_mars \
        src/px4_msgs \
        src/px4_ros_com \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        px4_msgs \
        px4_ros_com \
      --mixin $OVERLAY_MIXINS

COPY . ./src/carbodrone/carbodrone_mars
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        carbodrone_mars \
      --mixin $OVERLAY_MIXINS

# source entrypoint setup
ENV OVERLAY_WS=$OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

RUN useradd -m -d /home/user user
USER user

# run launch file
CMD ["ros2", "run", "carbodrone_mars", "ball_detector"]
