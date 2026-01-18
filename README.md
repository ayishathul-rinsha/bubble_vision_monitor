# Bubble Vision Monitor

ROS 2 Humble bubble detection and liquid color monitoring system with real-time web dashboard.

## Features
- Advanced bubble detection with adaptive lighting
- 60+ detailed color classifications (Blood Red, Navy Blue, Emerald Green, etc.)
- Real-time WebSocket dashboard
- Responsive UI (desktop/tablet/mobile)
- Change detection tracking with "previous -> current" notifications

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Webcam

## Installation
```bash
# Install dependencies
sudo apt update
sudo apt install -y python3-pip python3-opencv ros-humble-cv-bridge ros-humble-vision-opencv
pip3 install opencv-python websockets

# Clone repository
cd ~
git clone https://github.com/ayishathul-rinsha/bubble_vision_monitor.git
cd bubble_vision_monitor

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the System

Open **3 separate terminals** and run:

**Terminal 1 - Camera Node:**
```bash
cd ~/bubble_vision_monitor
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 install/camera_vision/bin/camera_node
```

**Terminal 2 - Vision Processor:**
```bash
cd ~/bubble_vision_monitor
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 install/camera_vision/bin/vision_processor
```

**Terminal 3 - Web Server:**
```bash
cd ~/bubble_vision_monitor
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 install/vision_web/bin/web_server
```

**Then open browser:** http://localhost:8000

## System Architecture
```
┌─────────────┐      ┌──────────────────┐      ┌─────────────┐
│ Camera Node │─────▶│ Vision Processor │─────▶│ Web Server  │
└─────────────┘      └──────────────────┘      └─────────────┘
                              │                        │
                              │                        ▼
                              ▼                  WebSocket (9000)
                    /camera/processed_image            │
                    /vision/bubbles                    ▼
                    /vision/color_status         Dashboard (8000)
```

## ROS 2 Topics

- `/camera/image_raw` - Raw camera feed (sensor_msgs/Image)
- `/camera/processed_image` - Processed feed with overlays (sensor_msgs/Image)
- `/vision/bubbles` - Bubble detection data (std_msgs/String as JSON)
- `/vision/color_status` - Color classification data (std_msgs/String as JSON)

## Adjustable Parameters
```bash
# Adjust bubble size detection
ros2 param set /vision_processor bubble_min_radius 5
ros2 param set /vision_processor bubble_max_radius 80

# View current parameters
ros2 param list /vision_processor
```

## Color Classifications

The system detects 60+ distinct colors including:
- **Reds:** Blood Red, Crimson, Deep Red, Burgundy, Dark Maroon
- **Oranges:** Bright Orange, Burnt Orange, Dark Orange
- **Yellows:** Lemon Yellow, Gold, Mustard Yellow, Golden Yellow
- **Greens:** Emerald Green, Forest Green, Lime Green, Sage Green
- **Blues:** Navy Blue, Royal Blue, Cobalt Blue, Sky Blue, Azure Blue
- **Purples:** Violet, Indigo, Lavender Purple, Periwinkle
- **Grays:** Charcoal, Silver, Medium Gray, Light Gray
- **Pastels:** Pale Pink, Peach, Mint Green, Lavender
- And many more!

## Troubleshooting

**Camera not detected:**
```bash
ls -l /dev/video*
sudo usermod -a -G video $USER  # Log out and back in
```

**Different camera device:**
```bash
# Edit camera_node to try /dev/video1, /dev/video2, etc.
```

**Slow performance:**
- System already optimized (processes every 2nd frame for bubbles)
- Lower camera resolution if needed

## Repository
https://github.com/ayishathul-rinsha/bubble_vision_monitor

## License
MIT
