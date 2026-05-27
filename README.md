# nrs_media

`nrs_media`는 ROS 2 Humble 기반의 MediaPipe/RealSense 비전 워크스페이스입니다. RealSense 카메라에서 얼굴, 손, 상체 랜드마크를 추출하고, 이를 ROS 토픽으로 발행하거나 로봇 손/몸체 제어 명령으로 변환합니다.

이 README는 conda 가상 환경 생성부터 의존성 설치, 워크스페이스 빌드, 노드 실행까지 한 번에 따라 할 수 있도록 정리되어 있습니다.

## 패키지 구성

```text
nrs_media/
├── README.md
└── src/
    ├── nrs_mediapipe/
    │   └── nrs_mediapipe/realsense_mediapipe_pose.py
    ├── nrs_hand_teleop/
    │   └── nrs_hand_teleop/hand_teleop_node.py
    ├── face_gesture/
    │   └── face_gesture/
    │       ├── face_gesture_node.py
    │       ├── face_gesture_node_v1.py
    │       ├── face_gesture_node_v2.py
    │       ├── face_gesture_node_v3.py
    │       └── face_gesture_v1.py
    └── nrs_media_bringup/
```

주요 패키지 역할:

- `nrs_mediapipe`: RealSense + MediaPipe Hands/Pose 인식, 손/상체 랜드마크 발행
- `nrs_hand_teleop`: MediaPipe 손 랜드마크를 로봇 손 관절 목표값으로 변환
- `face_gesture`: RealSense + MediaPipe Hands/Face Mesh 기반 얼굴/손 제스처 인식
- `nrs_media_bringup`: launch/config 통합용 예약 패키지

## 요구 환경

권장 환경:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10 conda 환경
- Intel RealSense 카메라, 예: D435
- Intel RealSense SDK 2.0 또는 `pyrealsense2`
- GUI 세션 또는 X11 forwarding 환경, `cv2.imshow()` 창 표시 필요
- 선택: `dualarm_ws`, `dualarm_forcecon`, `dualarm_forcecon_interfaces`

ROS 패키지 의존성:

- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `dualarm_forcecon_interfaces` (`nrs_hand_teleop` 실행 시 필요)
- `ament_python`
- 테스트용: `ament_copyright`, `ament_flake8`, `ament_pep257`, `pytest`

Python/pip 의존성:

- `mediapipe`
- `opencv-python`
- `numpy`
- `pyrealsense2`
- `setuptools`
- `colcon-common-extensions`
- `catkin_pkg`
- `empy`
- `lark`
- `pytest`

## 1. ROS 2 Humble 설치 확인

ROS 2 Humble이 이미 설치되어 있다고 가정합니다. 설치 여부를 확인합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

필요한 기본 ROS/빌드 패키지를 설치합니다.

```bash
sudo apt update
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-ament-copyright \
    ros-humble-ament-flake8 \
    ros-humble-ament-pep257
```

`rosdep`를 처음 쓰는 장비라면 한 번만 초기화합니다.

```bash
sudo rosdep init
rosdep update
```

이미 초기화되어 있다면 `sudo rosdep init`은 에러가 날 수 있으며, 그 경우 `rosdep update`만 실행하면 됩니다.

## 2. RealSense 준비

RealSense 카메라가 연결되어 있어야 하며, Python에서 `pyrealsense2`를 import할 수 있어야 합니다. 보통은 pip 패키지로 충분하지만, 장비에 따라 Intel RealSense SDK 2.0 설치가 먼저 필요할 수 있습니다.

카메라 연결 확인:

```bash
lsusb | grep -i realsense
```

RealSense Viewer가 설치되어 있다면 카메라 스트림도 확인합니다.

```bash
realsense-viewer
```

## 3. conda 환경 생성

이 워크스페이스는 `env_hand` conda 환경에서 빌드하고 실행하는 것을 기준으로 합니다.

```bash
conda create -n env_hand python=3.10 -y
conda activate env_hand
python -m pip install --upgrade pip setuptools wheel
```

Python 라이브러리를 설치합니다.

```bash
python -m pip install \
    mediapipe \
    opencv-python \
    numpy \
    pyrealsense2 \
    colcon-common-extensions \
    catkin_pkg \
    empy \
    lark \
    pytest
```

설치 확인:

```bash
python -c "import cv2, mediapipe, numpy, pyrealsense2; print('Python dependencies OK')"
```

중요: 이 워크스페이스는 반드시 `conda activate env_hand`가 된 상태에서 `colcon build`해야 합니다. 그렇지 않으면 `ros2 run` entry point가 시스템 Python을 사용해 `ModuleNotFoundError: No module named 'mediapipe'`가 발생할 수 있습니다.

## 4. 외부 워크스페이스 의존성

`nrs_hand_teleop`는 다음 인터페이스를 import합니다.

```python
from dualarm_forcecon_interfaces.srv import SetControlMode
```

따라서 `nrs_hand_teleop`를 빌드/실행하려면 `dualarm_forcecon_interfaces`가 설치되어 있거나, 해당 패키지가 들어 있는 워크스페이스를 먼저 source해야 합니다.

예시:

```bash
source ~/dualarm_ws/install/setup.bash
```

`dualarm_ws`가 없는 상태에서 전체 빌드를 하면 `dualarm_forcecon_interfaces`를 찾지 못해 빌드가 실패할 수 있습니다. 이 경우 아래처럼 해당 패키지를 제외하고 빌드할 수 있습니다.

```bash
colcon build --packages-skip nrs_hand_teleop
```

## 5. 워크스페이스 빌드

매 터미널에서 기본 순서는 conda 활성화 후 ROS 환경 source입니다.

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
```

`dualarm_forcecon_interfaces`가 필요한 경우 외부 워크스페이스도 source합니다.

```bash
source ~/dualarm_ws/install/setup.bash
```

빌드:

```bash
cd ~/nrs_media
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

`dualarm_ws` 없이 비전 패키지만 빌드:

```bash
cd ~/nrs_media
rosdep install --from-paths src --ignore-src -r -y --skip-keys dualarm_forcecon_interfaces
colcon build --packages-skip nrs_hand_teleop
source install/setup.bash
```

이미 잘못된 Python 환경에서 빌드했다면 산출물을 지우고 다시 빌드합니다.

```bash
cd ~/nrs_media
rm -rf build install log
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
colcon build
source install/setup.bash
```

## 6. 실행 전 터미널 설정

`nrs_media` 노드를 실행하는 터미널에서는 다음 순서를 사용합니다.

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/nrs_media/install/setup.bash
```

`nrs_hand_teleop` 또는 `dualarm_forcecon_interfaces`가 필요한 노드는 다음처럼 외부 워크스페이스를 먼저 source합니다.

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
source ~/nrs_media/install/setup.bash
```

## 7. nrs_mediapipe 실행

RealSense 색상/깊이 프레임을 읽고 MediaPipe Hands/Pose를 실행합니다.

```bash
ros2 run nrs_mediapipe realsense_mediapipe_pose
```

주요 발행 토픽:

- `/mediapipe_hand_landmarks` (`std_msgs/msg/String`, JSON)
- `/upper_body_landmarks` (`std_msgs/msg/String`, JSON)
- `/hand_gesture` (`std_msgs/msg/String`)
- `/target_pose` (`geometry_msgs/msg/Point`)
- `/guided_target` (`geometry_msgs/msg/Point`)
- `/surface_command` (`std_msgs/msg/String`)

구독 토픽:

- `/robot_status` (`std_msgs/msg/String`)

확인:

```bash
ros2 topic echo /mediapipe_hand_landmarks
ros2 topic echo /upper_body_landmarks
```

OpenCV 창이 열리며, `q`를 누르면 종료됩니다.

## 8. nrs_hand_teleop 실행

`/mediapipe_hand_landmarks`를 받아 로봇 손 30차원 목표 관절값을 발행합니다.

먼저 `nrs_mediapipe` 노드가 실행 중이어야 합니다.

```bash
ros2 run nrs_hand_teleop hand_teleop_node
```

주요 인터페이스:

- 구독: `/mediapipe_hand_landmarks` (`std_msgs/msg/String`, JSON)
- 발행: `/forward_hand_joint_targets` (`std_msgs/msg/Float64MultiArray`)
- 서비스 클라이언트: `/change_control_mode` (`dualarm_forcecon_interfaces/srv/SetControlMode`)

동작:

- 왼손 15개 + 오른손 15개 = 총 30개 손 관절 목표값 발행
- `/change_control_mode` 서비스로 `arm_mode=idle`, `hand_mode=forward` 요청

확인:

```bash
ros2 topic echo /forward_hand_joint_targets
```

## 9. face_gesture 실행

`face_gesture` 패키지는 RealSense + MediaPipe Face Mesh/Hands 기반 얼굴/손 제스처 노드입니다.

빌드된 ROS entry point:

```bash
ros2 run face_gesture face_gesture_node
```

이 entry point는 `src/face_gesture/face_gesture/face_gesture_node.py`를 실행합니다.

주요 발행 토픽:

- `/left_hand` (`geometry_msgs/msg/Point`)
- `/right_hand` (`geometry_msgs/msg/Point`)
- `/face_pose` (`geometry_msgs/msg/Point`)
- `/face_gesture_cmd` (`std_msgs/msg/String`)

`face_gesture_node_v1.py`, `face_gesture_node_v2.py`, `face_gesture_node_v3.py`, `face_gesture_v1.py`는 현재 `setup.py`의 console script로 등록되어 있지 않습니다. 필요한 경우 Python 모듈로 직접 실행하거나 `setup.py`에 entry point를 추가해야 합니다.

직접 실행 예시:

```bash
cd ~/nrs_media/src/face_gesture
python -m face_gesture.face_gesture_node_v3
```

`face_gesture_node_v2.py`와 `face_gesture_node_v3.py`는 추가로 `/cmd_vel`, `/forward_aux_joint_targets` 같은 로봇 제어 토픽을 발행합니다. 로봇에 연결된 상태에서 실행할 때는 토픽 연결 상태를 먼저 확인하세요.

## 10. 권장 실행 순서

로봇 컨트롤러까지 포함하는 경우:

터미널 1, 로봇 컨트롤러:

```bash
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

터미널 2, MediaPipe 인식:

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/nrs_media/install/setup.bash
ros2 run nrs_mediapipe realsense_mediapipe_pose
```

터미널 3, 손 teleop:

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
source ~/nrs_media/install/setup.bash
ros2 run nrs_hand_teleop hand_teleop_node
```

비전 제스처 노드만 실행하는 경우:

```bash
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/nrs_media/install/setup.bash
ros2 run face_gesture face_gesture_node
```

## 11. 테스트와 점검 명령

패키지 목록 확인:

```bash
ros2 pkg list | grep -E 'nrs_mediapipe|nrs_hand_teleop|face_gesture|nrs_media_bringup'
```

실행 파일 확인:

```bash
ros2 pkg executables nrs_mediapipe
ros2 pkg executables nrs_hand_teleop
ros2 pkg executables face_gesture
```

토픽 확인:

```bash
ros2 topic list
ros2 topic info /mediapipe_hand_landmarks
ros2 topic info /forward_hand_joint_targets
```

테스트 실행:

```bash
cd ~/nrs_media
colcon test
colcon test-result --verbose
```

## 12. 문제 해결

### `ModuleNotFoundError: No module named 'mediapipe'`

대부분 워크스페이스를 conda 환경이 아닌 시스템 Python으로 빌드했을 때 발생합니다.

```bash
cd ~/nrs_media
rm -rf build install log
conda activate env_hand
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
colcon build
source install/setup.bash
```

### `dualarm_forcecon_interfaces`를 찾을 수 없음

`nrs_hand_teleop`는 외부 서비스 타입이 필요합니다.

```bash
source ~/dualarm_ws/install/setup.bash
```

외부 워크스페이스 없이 비전 노드만 사용할 경우:

```bash
colcon build --packages-skip nrs_hand_teleop
```

### RealSense frame timeout

예시:

```text
RuntimeError: Frame didn't arrive within 5000
```

확인할 항목:

- USB 3.x 포트에 직접 연결되어 있는지
- 다른 프로세스가 카메라를 사용 중인지
- `realsense-viewer`에서 스트림이 정상인지
- 케이블 또는 허브 전원이 안정적인지

### OpenCV 창이 뜨지 않음

이 노드들은 `cv2.imshow()`를 사용합니다. SSH 환경에서는 X11 forwarding 또는 실제 디스플레이 세션이 필요합니다.

```bash
echo $DISPLAY
```

값이 비어 있으면 GUI 창을 띄울 수 없습니다.

### `ros2 run`은 실패하지만 `python -m ...`은 동작함

빌드 시점의 Python interpreter와 현재 conda Python이 다를 가능성이 큽니다. `env_hand`를 활성화한 뒤 clean build 하세요.

## 13. 현재 구현 상태

구현됨:

- RealSense + MediaPipe Hands/Pose 인식
- RealSense + MediaPipe Face Mesh 기반 얼굴/손 제스처 인식
- `/mediapipe_hand_landmarks`, `/upper_body_landmarks`, `/hand_gesture` 발행
- `/mediapipe_hand_landmarks` 기반 30차원 손 teleop 목표값 발행
- 일부 face gesture 노드의 `/cmd_vel`, `/forward_aux_joint_targets` 발행

추가 권장 작업:

- `nrs_media_bringup`에 launch 파일 추가
- YAML 파라미터 파일 추가
- JSON 문자열 대신 custom ROS message 도입
- `face_gesture_node_v1/v2/v3` entry point 정리
