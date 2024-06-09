# Carbodrone

## Installing

Good luck

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
