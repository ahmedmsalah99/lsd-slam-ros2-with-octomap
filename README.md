# LSD-SLAM ROS 2 with OctoMap Integration

This project integrates **LSD-SLAM** running in a Docker container with a **ROS 2 bridge** and **OctoMap server**, allowing real-time mapping from monocular vision. Data exchange is handled via sockets, and the results are visualized in RViz.

---

## 🧠 Project Overview

- LSD-SLAM runs in a **Docker container**.
- Host communicates with the container using two sockets:
  - **Port 9000**: Grayscale image input from host to container.
  - **Port 9002**: Keyframe point cloud output from container to host.
- A **ROS 2 bridge node** receives image/point cloud data and publishes it in ROS 2.
- An **OctoMap server** subscribes to the `/livox/lidar` topic and builds a 3D occupancy map.

---

## 📦 Build Instructions

### Clone & Import Dependencies

```bash
cd octomap_ws/octomap_server2
vcs import . < deps.repos
cd ../
```

### Install Dependencies

```bash
rosdep install --from-paths . --ignore-src -r -y
sudo apt install ros-humble-message-filters
```

### Build OctoMap packages

```bash
colcon build --symlink-install --packages-select octomap_msgs octomap_server2
```

---

## 🖼️ LSD-SLAM Bridge Setup

> Make sure you have already built and are ready to run the LSD-SLAM container. Instructions for this are in the [`lsd_slam_ros2`](https://github.com/your-org/lsd_slam_ros2) repository.

### Optional: Download Example Dataset

```bash
export SEQUENCE=30  # Sequence number of the TUM mono dataset
./scripts/download_tum_mono.sh
```

This step is only needed if you plan to use the `dummy_video_pub_node`. Alternatively, you can publish your own grayscale images to `/camera/image`.

### Build the bridge

```bash
cd ros2_ws
colcon build
source install/setup.sh
```

---

## 🚀 Run the System

### 1. Start LSD-SLAM container
Run `main_on_images_sock` inside the container. See LSD-SLAM repo for details.

### 2. On host: run the bridge node

```bash
ros2 run lsd_slam_bridge lsd_slam_bridge_node
```

### 3. (Optional) Publish images from dataset

```bash
ros2 run lsd_slam_bridge dummy_video_pub_node --ros-args -p image_folder:="../lsd_slam_ros2/data/sequence_30"
```

---

## 🗺️ Visualization

- The bridge node publishes the keyframe point clouds to `/livox/lidar`.
- The OctoMap server subscribes to `/livox/lidar` and builds a map.
- You can visualize both the point cloud and OctoMap in **RViz**.

---

## ⚠️ Limitations

1. **Real-Time Performance**:
   - LSD-SLAM is sensitive to image frame rates. 
   - Each frame currently takes ~800ms to be processed, which makes real-time tracking difficult.

2. **Map Consistency**:
   - The current system publishes only the keyframe-based belief of the map.
   - A more robust map would require publishing a denser or fused global point cloud.

---

## 🧩 Acknowledgements

- [TUM Mono dataset](https://vision.in.tum.de/data/datasets/mono-dataset)
- [LSD-SLAM no ros](https://github.com/IshitaTakeshi/lsd_slam_noros)
- [OctoMap server2](https://github.com/iKrishneel/octomap_server2)

---
