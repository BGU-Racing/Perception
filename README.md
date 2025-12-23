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
│   └── ros/
│       └── src/
│           └── perception/
│               ├── perception/
│               │   ├── camera/
│               │   ├── lidar/
│               │   │   ├── algo/
│               │   │   │   ├── calibration/
│               │   │   │   │   └── python/
│               │   │   │   └── detection/
│               │   │   │       └── python/
│               │   │   ├── readers/
│               │   │   │   └── python/
│               │   │   └── visualizers/
│               │   └── nodes/
│               ├── resource/
│               └── test/
├── tests/
├── utils/
└── README.md
```

## Development Environment (Docker + ROS 2 Jazzy)

### 1. Build the Docker image
From the `deployment/` directory:

```
cd deployment
make build
```

You may also build manually:

```
docker build -t ros2-jazzy-dev-image -f Dockerfile .
```

---

### 2. Run the container
Using the Makefile:

```
make run
```

This will create (or start) a container named:

```
ros2-jazzy-dev-container
```

Your host repository root is mounted into the container at:

```
/workspace
```

---

### 3. Attach to the container

```
make attach
```

Or manually:

```
docker attach ros2-jazzy-dev-container
```

---

### 4. Stop and remove the container

```
make stop
make clean
```

---

## Inside the Container

ROS 2 and the Python virtual environment are automatically sourced.

You can run ROS and development commands normally:

```
ros2 run <package> <node>
ros2 topic list
python3
colcon build
```

---

## Repository Structure

```
Perception/
│
├── deployment/
│
├── src/           
│   ├── camera/
│   ├── lidar/
│   └── ros/
│
├── tests/
│
└── README.md
```

---

## Getting Started

```
source /workspace/src/ros/install/setup.bash
```

5. Run your perception nodes.

---

## Notes

- ROS 2 Jazzy is Ubuntu 24.04 (Noble) based.
- Python environment includes NumPy, OpenCV, Open3D, Scikit‑Learn, and Ultralytics (YOLO11).
- Your container maps directly to your local filesystem: all changes persist.

---
