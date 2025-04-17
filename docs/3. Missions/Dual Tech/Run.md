## Run

In multiple terminals:

```bash
MicroXRCEAgent udp4 -p 8888
```

```bash
cd ~/px4/PX4-gazebo-models
./simulation-gazebo --world dualtech
```

```bash
cd ~/px4/PX4-Autopilot
PX4_GZ_MODEL_POSE=-6,-6 PX4_GZ_STANDALONE=1 make px4_sitl gz_x500_depth
```

```bash
cd ~/ros2_ws
ros2 launch carbodrone_px sim.launch.py
```
