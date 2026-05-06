# 🤖 Vision-Based Gesture Controller for Dual-Arm Robot (`face_gesture_node`)

This package is a ROS 2 node that utilizes an **Intel RealSense camera** and **MediaPipe** to recognize user face gestures and hand movements in real-time, directly controlling the hand joints of a Dual-Arm Robot.

Going beyond simple vision tracking, it acts as a **"Vision-to-Robot Controller"** by issuing direct control commands to the robot's `forward_hand_joint_targets` topic based on your gestures.

## 🌟 Key Features

*   **3D Spatial Recognition (RealSense + MediaPipe)**: Combines 2D pixel coordinates with Depth data to estimate and publish real 3D spatial coordinates (x, y, z) of the user's hands and face.
*   **Custom User Calibration**: During the first 5 seconds after launch, it learns the user's neutral Eye Aspect Ratio (EAR) and eyebrow height ratio to maximize recognition accuracy.
*   **State Machine-Based Control**: When a specific gesture is triggered, the system state becomes 'LOCKED' to prevent unintended multiple inputs. It can only be reset to the 'IDLE' state by holding a 'Right Fist' for 3 seconds.
*   **Intuitive UI Feedback**: Real-time text overlay on the OpenCV display shows the currently triggered gesture and the actual command sent to the robot (e.g., `Wink Right! : Left hand open, Right hand close`).

## 🕹️ Gesture-to-Command Mapping

To control the 40 joints of the robot's dual hands (20 per hand), this node publishes specific `Float64MultiArray` data when a gesture is confirmed.

| Gesture | Trigger Condition | Robot Action | UI Feedback Output |
| :--- | :--- | :--- | :--- |
| **Left Wink** (2.0s) | Left EAR < 65% | **Command 6**: Left hand close, Right hand open | `Wink Left! : Left hand close...` |
| **Right Wink** (2.0s) | Right EAR < 65% | **Command 7**: Left hand open, Right hand close | `Wink Right! : Left hand open...` |
| **Mouth Open** (3.0s) | Mouth Open Ratio > 5% | **Command 5**: Close both hands strongly | `Mouth Open! : Close both hands strongly` |
| **Right Fist** (3.0s) | Fingertips tucked in | **Command 1 (RESET)**: Return to IDLE & open both hands | `IDLE! : Open both hands` |

*(Note: The `TURN_LEFT`, `TURN_RIGHT`, and `EYEBROW_RAISE` gestures are successfully recognized and will lock the state, but they are currently "Reserved" and do not yet publish joint commands.)*

## 📡 ROS 2 Interfaces (Topics)

### Publishers
*   `/left_hand` (`geometry_msgs/msg/Point`): 3D coordinates of the left hand.
*   `/right_hand` (`geometry_msgs/msg/Point`): 3D coordinates of the right hand.
*   `/face_pose` (`geometry_msgs/msg/Point`): 3D coordinates of the face.
*   `/face_gesture_cmd` (`std_msgs/msg/String`): String command of the triggered gesture (e.g., `WINK_LEFT`, `IDLE`).
*   `/forward_hand_joint_targets` (`std_msgs/msg/Float64MultiArray`): 40-DOF robot hand joint control data (for v31 Hand Controller communication).

## 🛠️ Dependencies

*   **ROS 2** (Humble recommended)
*   **Python 3.x**
*   **OpenCV** (`cv2`)
*   **MediaPipe** (`mediapipe`)
*   **Intel RealSense SDK 2.0** (`pyrealsense2`)

## 🚀 Usage

1.  Ensure your Intel RealSense camera is properly connected to your PC.
2.  Build your ROS 2 workspace (e.g., `dualarm_ws`) and source the environment.
3.  Run the node using the following command:
    ```bash
    python3 face_gesture_node.py
    # Or if built as a ROS 2 package:
    ros2 run <your_package_name> face_gesture_node
    ```
4.  Once the camera feed opens, **keep a neutral face for the first 5 seconds** to complete the calibration process.
5.  Wait until the text `State: IDLE (Ready for Gesture)` appears in the top-left corner, then perform your gestures.
6.  To exit, click on the display window and press the `q` key, or press `Ctrl+C` in the terminal.
