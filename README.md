# 🗺️ ROS 2 Map Compression

**ros2-map-compression** is a lightweight ROS 2 Python package for **compressing and decompressing occupancy grid maps** (`nav_msgs/OccupancyGrid`) using the efficient, lossless **zlib** algorithm. It’s designed for multi-robot mapping, SLAM data transmission over limited bandwidth, and reducing map memory usage on embedded systems.

---

## 🚀 Features

-  Compresses the `/map` topic in real-time using zlib
-  Decompresses to `/decompressed_map` with full map fidelity
- Built using standard `rclpy`, no custom messages
- Compare original and decompressed maps in RViz
-  No YAML or external config — minimal and self-contained

---

##  Package Layout


---

##  How It Works

1. **`map_compressor.py`**
   - Subscribes to `/map`
   - Compresses the `OccupancyGrid` message’s binary content using `zlib`
   - Publishes it as a `std_msgs/String` message on `/compressed_map`

2. **`map_decompressor.py`**
   - Subscribes to `/compressed_map`
   - Decompresses it using `zlib`
   - Publishes the restored message as a `nav_msgs/OccupancyGrid` on `/decompressed_map`

---

## 🧪 Topic Overview

| Topic                | Type                      | Description                                |
|---------------------|---------------------------|--------------------------------------------|
| `/map`              | `nav_msgs/OccupancyGrid`  | The raw map published by SLAM              |
| `/compressed_map`   | `std_msgs/String`         | The zlib-compressed string version         |
| `/decompressed_map` | `nav_msgs/OccupancyGrid`  | Reconstructed map after decompression      |

---

## 🛠️ Usage Instructions

### 1. Clone the Repo and Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/imdsafi09/ros2-map-compression.git
cd ..
colcon build --packages-select ros2-map-compression
source install/setup.bash
ros2 launch ros2-map-compression map_compression.launch.py
rviz2
ros2 run ros2-map-compression map_comparator_plot
