# 4WD Skid-Steer Scouting Rover
ROS2 Humble | Gazebo Fortress | Python 3.10 | WSL2 

---

## Problem Statement

Post-earthquake urban environments are too dangerous for humans to enter immediately due to collapsed buildings, confined areas, and unstable terrain. Rescue workers need to be familiar with the area before being sent.

## Solution

Using SLAM, a simulated 4WD skid-steer rover can autonomously navigate disaster terrain and generate a real-time 2D occupancy map, allowing rescue personnel to assess the area from a distance before physically entering.
---

## Stack

| Component | Detail |
|-----------|--------|
| Platform | Panther v0.2 (GrabCAD) |
| Middleware | ROS2 Humble |
| Simulator | Gazebo Fortress (Ignition 6.17.1) |
| Environment | Ubuntu 22.04 / WSL2 + WSLg |

---

## Sensors

| Sensor | Topic | Rate |
|--------|-------|------|
| 360-degree LiDAR (0.15–12 m) | `/scan` | 10 Hz |
| RGB Camera (640x480) | `/camera/image_raw` | 30 Hz |
| 6-DOF IMU | `/imu/data` | 100 Hz |

---

## Quick Start

```bash
cd ~/ros2_ws
colcon build --packages-select rover_description
source install/setup.bash

# Launch simulation + SLAM + RViz
ros2 launch rover_description slam.launch.py rviz:=true

# Run teleop in a new terminal
ros2 run rover_description teleop_hold_node
```

**WSL2 only — set display first:**
```bash
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
```

---

## Controls

| Key | Action |
|-----|--------|
| Arrow keys (hold) | Move / Rotate |
| SPACE | Toggle hold-position mode |
| q / z | Speed up / down |
| s | Save map to disk |
| CTRL+C | Quit |

---

## Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/rover_map
```

---

