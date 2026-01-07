# Perception

BGRacing Perception Team – LiDAR & Camera 

## Overview
This repository contains the perception stack for the BGRacing Formula Student team.  
It provides a ROS 2 Jazzy–based development environment for working with LiDAR, camera modules.

All development is intended to run inside the provided Docker environment.

---

```
PERCEPTION/
├── data/
├── deployment/
│   ├── Makefile
│   └── Dockerfile
├── src/
│   └── perception/
│        ├── perception/
│        │   ├── camera/
│        │   ├── lidar/
│        │   │   ├── algo/
│        │   │   │   ├── calibration/
│        │   │   │   │   └── python/
│        │   │   │   └── detection/
│        │   │   │       └── python/
│        │   │   ├── readers/
│        │   │   │   └── python/
│        │   │   └── visualizers/
│        │   └── nodes/
│        └── test/
├── tests/
├── utils/
└── README.md
```

## Development Environment 

### 1. Build the Docker image
From the `root/` directory:

```
make build
```

### 2. Run the container
Using the Makefile:

```
make run
```

This will start a container named:

```
ros2-jazzy-dev-container
```

Your host repository root is mounted into the container at:

```
/workspace
```

```
cd Perception
```

---

### 3. Attach, exit and delete the container

```
make attach
exit
make clean
```

---

## 5. Inside the Container

Once inside the container, ROS 2 Jazzy and the Python virtual environment are already sourced.

### Build the ROS workspace

Navigate to the ROS workspace and build all packages:

```
cd /Perception/src
colcon build --symlink-install
```

After building, source the workspace:

```
source install/setup.bash
```

Run the perception pipeline

```
ros2 launch perception PCLPipeline.launch.py
```

Run individual nodes
```
ros2 run perception pcl_recording_reader
ros2 run perception pcl_preprocess
ros2 run perception pcl_detection
```

## Notes

- ROS 2 Jazzy is Ubuntu 24.04 (Noble) based.
- Python environment includes NumPy, OpenCV, Open3D, Scikit‑Learn, and Ultralytics (YOLO11).
- Your container maps directly to your local filesystem: all changes persist.

---
