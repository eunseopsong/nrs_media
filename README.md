# nrs_media

`nrs_media` is a ROS 2 Humble workspace for MediaPipe-based human hand perception and robot-hand teleoperation.

This repository currently focuses on:

- **`nrs_mediapipe`**: RealSense + MediaPipe perception
- **`nrs_hand_teleop`**: converting MediaPipe hand landmarks into robot hand joint targets
- **`nrs_media_bringup`**: reserved package for launch/config integration

The current workflow is:

```text
RealSense (D435)
    -> MediaPipe Hands / Pose
    -> /mediapipe_hand_landmarks
    -> nrs_hand_teleop
    -> /forward_hand_joint_targets
    -> dualarm_forcecon
```

---

# 1. Repository Structure Summary

## Workspace structure

```text
nrs_media/
├── src/
│   ├── nrs_mediapipe/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── nrs_mediapipe/
│   │       ├── __init__.py
│   │       └── realsense_mediapipe_pose.py
│   │
│   ├── nrs_hand_teleop/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   └── nrs_hand_teleop/
│   │       ├── __init__.py
│   │       └── hand_teleop_node.py
│   │
│   └── nrs_media_bringup/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       └── nrs_media_bringup/
│           └── __init__.py
│
├── build/
├── install/
├── log/
└── README.md
```

## Package roles

### `nrs_mediapipe`
Perception package.

Main responsibilities:
- capture color/depth frames from RealSense
- run MediaPipe Hands and Pose
- keep existing hand gesture / upper-body visualization logic
- publish:
  - `/mediapipe_hand_landmarks`
  - `/upper_body_landmarks`
  - `/hand_gesture`
  - existing target / guided topics if needed

Main node:
- `realsense_mediapipe_pose`

### `nrs_hand_teleop`
Teleoperation package.

Main responsibilities:
- subscribe to `/mediapipe_hand_landmarks`
- compute finger flexion from MediaPipe hand landmarks
- convert human hand motion into robot hand joint targets
- publish:
  - `/forward_hand_joint_targets`
- request control mode:
  - `arm_mode = idle`
  - `hand_mode = forward`

Main node:
- `hand_teleop_node`

### `nrs_media_bringup`
Reserved integration package.

Recommended future usage:
- launch files
- YAML config
- full demo orchestration

---

# 2. Dependency: Conda Environment and Python Library Installation

## 2-1. Prerequisites

Required system-level software:

- Ubuntu 22.04
- ROS 2 Humble
- Intel RealSense SDK / `pyrealsense2`
- a working conda installation
- optional but recommended: `dualarm_ws` already built, because `nrs_hand_teleop` uses `dualarm_forcecon_interfaces`

---

## 2-2. Create the conda environment

This workspace is intended to run with the **`env_hand`** conda environment.

Create it:

```bash
conda create -n env_hand python=3.10 -y
conda activate env_hand
```

---

## 2-3. Install Python libraries

Install the Python packages used by MediaPipe / RealSense / ROS Python nodes:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install \
    mediapipe \
    opencv-python \
    numpy \
    pyrealsense2 \
    colcon-common-extensions \
    catkin_pkg \
    empy \
    lark \
    setuptools
```

If `pyrealsense2` installation fails through pip on your machine, install RealSense SDK first and then verify import manually.

Verify Python-side imports:

```bash
python3 -c "import mediapipe, cv2, numpy, pyrealsense2; print('Python dependencies OK')"
```

---

## 2-4. ROS 2 environment

Every terminal used for this repository should source ROS 2 Humble first:

```bash
source /opt/ros/humble/setup.bash
```

If you use `dualarm_forcecon` and `dualarm_forcecon_interfaces` from another workspace, also source that workspace before building or running `nrs_hand_teleop`:

```bash
source ~/dualarm_ws/install/setup.bash
```

---

## 2-5. Build the workspace

From the workspace root:

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash   # if needed

cd ~/nrs_media
colcon build
source install/setup.bash
```

### Important note about Python interpreter
This workspace should be built **while `env_hand` is activated**.

Otherwise, `ros2 run` may use `/usr/bin/python3` instead of the conda Python, which can cause errors like:

```text
ModuleNotFoundError: No module named 'mediapipe'
```

If needed, clean and rebuild:

```bash
cd ~/nrs_media
rm -rf build install log
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash   # if needed
colcon build
source install/setup.bash
```

---

# 3. Usage: Conda Activation and Node Execution

## 3-1. Terminal setup rule

For all terminals, use this basic order:

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/nrs_media/install/setup.bash
```

For teleoperation nodes that depend on `dualarm_forcecon_interfaces`, also source:

```bash
source ~/dualarm_ws/install/setup.bash
```

---

## 3-2. Run the MediaPipe perception node

This node captures RealSense images and publishes hand landmarks.

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/nrs_media/install/setup.bash

ros2 run nrs_mediapipe realsense_mediapipe_pose
```

Published topics include:

- `/mediapipe_hand_landmarks`
- `/upper_body_landmarks`
- `/hand_gesture`

You can check the hand landmark topic with:

```bash
ros2 topic echo /mediapipe_hand_landmarks
```

---

## 3-3. Run the hand teleoperation node

This node subscribes to `/mediapipe_hand_landmarks` and publishes `/forward_hand_joint_targets`.

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
source ~/nrs_media/install/setup.bash

ros2 run nrs_hand_teleop hand_teleop_node
```

Main behavior:
- reads left/right 21-point MediaPipe hand landmarks
- estimates finger flexion
- converts them into a 30D robot-hand target:
  - `left15 + right15`
- requests:
  - `arm_mode = idle`
  - `hand_mode = forward`

Check output:

```bash
ros2 topic echo /forward_hand_joint_targets
```

---

## 3-4. Run the robot control package

In another terminal, run your robot hand / dual-arm controller:

```bash
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## 3-5. Recommended execution order

### Terminal 1: robot controller
```bash
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

### Terminal 2: MediaPipe perception
```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/nrs_media/install/setup.bash
ros2 run nrs_mediapipe realsense_mediapipe_pose
```

### Terminal 3: hand teleoperation
```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
source ~/nrs_media/install/setup.bash
ros2 run nrs_hand_teleop hand_teleop_node
```

---

## 3-6. Recommended camera
The current setup was successfully tested with **RealSense D435**.

If you encounter unstable frame timeouts with another RealSense camera, verify:
- USB connection quality
- camera power stability
- RealSense SDK installation
- whether another process is already using the camera

---

## 3-7. Troubleshooting

### 1. `ModuleNotFoundError: No module named 'mediapipe'`
Cause:
- workspace was built with system Python instead of the conda Python

Fix:
```bash
cd ~/nrs_media
rm -rf build install log
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash   # if needed
colcon build
source install/setup.bash
```

### 2. `ros2 run` works differently from `python3 file.py`
Cause:
- installed entry-point may use a different interpreter than your current shell

Fix:
- always build inside `env_hand`
- always source `install/setup.bash` before running

### 3. RealSense frame timeout
Example:
```text
RuntimeError: Frame didn't arrive within 5000
```

Fix:
- reconnect the camera
- check USB 3.x connection
- close other processes using RealSense
- prefer D435 if it is more stable in your setup

---

# Current Status

Implemented:
- MediaPipe Hands + Pose perception
- `/mediapipe_hand_landmarks` publishing
- hand-only teleoperation to `/forward_hand_joint_targets`

Planned / recommended next steps:
- add launch files in `nrs_media_bringup`
- add YAML parameter files
- add custom ROS messages instead of `String(JSON)`
- extend from hand-only teleop to full upper-body / arm teleop
