# Bubble Vision Monitor

ROS 2 Humble bubble detection and liquid color monitoring system with real-time web dashboard.

## Features
- Advanced bubble detection with adaptive lighting
- 60+ detailed color classifications (Blood Red, Navy Blue, Emerald Green, etc.)
- Real-time WebSocket dashboard
- Responsive UI (desktop/tablet/mobile)
- Change detection tracking

## Quick Start
```bash
cd ~/bubble_vision_monitor
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch launch/bubble_vision_launch.py
```

Open browser: http://localhost:8000

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Webcam

## Repository
https://github.com/ayishathul-rinsha/bubble_vision_monitor
