# Carbodrone

## Installing

*Good luck*

1. Install ROS2 Humble

2. Install PX4 (installs Gazebo Garden)

    ```bash
    mkdir -p ~/px4
    cd ~/px4

    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    git clone https://github.com/PX4/PX4-gazebo-models.git

    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    ```

3. Install QGroundControl

    ```bash
    # https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu
    wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
    chmod +x QGroundControl.AppImage
    sudo mv QGroundControl.AppImage /usr/bin/QGroundControl
    ```

4. Configure `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`

5. Disable `SYNCT` in PX4 startup script

6. Configure custom gazebo worlds and models

7. ...

8. Compile

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/PX4/px4_msgs.git
    git clone https://github.com/PX4/px4_ros_com.git
    git clone https://github.com/AGH-Drone-Engineering/carbodrone.git
    cd ..
    colcon build --symlink-install
    ```

## Running

In multiple terminals:

```bash
MicroXRCEAgent udp4 -p 8888
```

```bash
./simulation-gazebo --world mars
```

```bash
PX4_GZ_MODEL_POSE=0,-9 make px4_sitl gz_x500_depth_mars
```

```bash
ros2 launch carbodrone_px main.launch.py
```
