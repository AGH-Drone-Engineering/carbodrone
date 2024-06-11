# Carbodrone

## Installing

*Good luck*

1. Install ROS2 Humble

2. Prepare ROS workspace

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/PX4/px4_msgs.git
    git clone https://github.com/PX4/px4_ros_com.git
    git clone https://github.com/AGH-Drone-Engineering/carbodrone.git
    ```

3. Install PX4 (installs Gazebo Garden)

    ```bash
    mkdir -p ~/px4
    cd ~/px4

    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    git clone https://github.com/PX4/PX4-gazebo-models.git

    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    ```

4. Install QGroundControl

    ```bash
    # https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu
    wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
    chmod +x QGroundControl.AppImage
    sudo mv QGroundControl.AppImage /usr/bin/QGroundControl
    ```

5. Configure `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`

    ```yaml
    subscriptions:
      - topic: /fmu/in/landing_target_pose
        type: px4_msgs::msg::LandingTargetPose
    ```

6. Disable `UXRCE_DDS_SYNCT` in PX4 startup script `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS`

    ```bash
    # before "uxrce_dds_client start" around line 310
    param set UXRCE_DDS_SYNCT 0
    ```

7. Configure custom gazebo worlds and models

    ```bash
    cd ~/px4/PX4-gazebo-models
    chmod +x simulation-gazebo
    ./simulation-gazebo
    # exit the program after starting
    ln -s $HOME/ros2_ws/src/carbodrone/**/worlds/*.sdf ~/.simulation-gazebo/worlds/
    # modify ~/.simulation-gazebo/models/
    # coming soon
    ```

8. Install uXRCE-DDS

    ```bash
    cd ~/px4
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
    ```

9. Install ROS dependencies

    ```bash
    # rosdep should work but it doesn't
    # install everything from all package.xml files
    ```

10. Compile

    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

## Running

In multiple terminals:

```bash
MicroXRCEAgent udp4 -p 8888
```

```bash
cd ~/px4/PX4-gazebo-models
./simulation-gazebo --world mars
```

```bash
cd ~/px4/PX4-Autopilot
PX4_GZ_MODEL_POSE=-6,-6 PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_depth
```

```bash
ros2 launch carbodrone_px main.launch.py
```
