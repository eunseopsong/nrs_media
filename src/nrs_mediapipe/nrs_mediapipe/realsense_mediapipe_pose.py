import sys
import cv2
import json
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import threading
import time


class UnifiedVisionNodeLightV7(Node):
    def __init__(self):
        super().__init__('nrs_vision_node_light_v7')

        self.gesture_pub = self.create_publisher(String, '/hand_gesture', 10)
        self.target_pub = self.create_publisher(Point, '/target_pose', 10)
        self.guided_pub = self.create_publisher(Point, '/guided_target', 10)
        self.surface_cmd_pub = self.create_publisher(String, '/surface_command', 10)

        # Added: upper body landmarks publisher
        self.upper_body_pub = self.create_publisher(String, '/upper_body_landmarks', 10)

        self.robot_state = "NOT READY"
        self.create_subscription(String, '/robot_status', self.status_cb, 10)

        self.surfaces = [
            "flat_surface_5.stl",
            "concave_surface_1.stl",
            "concave_surface_2.stl",
            "_concave_surface_0.75.stl",
            "_comp_concave_0_75_v0_42.stl",
            "compound_concave_0_75_v0_42.stl"
        ]
        self.current_idx = 0
        self.launch_triggered = False
        self.last_left_gesture = "None"

        # MediaPipe Hands (keep all existing gesture functionality)
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # Added: MediaPipe Pose for upper-body landmarks
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        try:
            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        except Exception as e:
            self.get_logger().error(f"RealSense Error: {e}")
            sys.exit(1)

        self.is_running = True
        self.vision_thread = threading.Thread(target=self.camera_loop)
        self.vision_thread.start()
        self.get_logger().info("✅ LIGHT Vision V7 Started (Hands + Upper-Body Pose)")

    def status_cb(self, msg):
        self.robot_state = msg.data

    def recognize_gesture(self, hand_landmarks, label):
        tips = [4, 8, 12, 16, 20]
        pips = [3, 6, 10, 14, 18]
        up = []

        if label == 'Right':
            up.append(hand_landmarks.landmark[4].x > hand_landmarks.landmark[3].x)
        else:
            up.append(hand_landmarks.landmark[4].x < hand_landmarks.landmark[3].x)

        for i in range(1, 5):
            up.append(hand_landmarks.landmark[tips[i]].y < hand_landmarks.landmark[pips[i]].y)

        dist = np.linalg.norm(np.array([
            hand_landmarks.landmark[8].x - hand_landmarks.landmark[4].x,
            hand_landmarks.landmark[8].y - hand_landmarks.landmark[4].y
        ]))

        if dist < 0.08 and up[2] and up[3] and up[4]:
            return "okay"
        if up[1] and up[2] and not up[3] and not up[4]:
            return "scissors"
        if up.count(True) >= 4:
            return "paper"
        if up.count(True) <= 1 and not up[1]:
            return "rock"
        if up[1] and not up[2]:
            return "pointing"
        return "None"

    def extract_upper_body_landmarks(self, pose_landmarks, img_w, img_h):
        """
        Extract only upper-body related landmarks from MediaPipe Pose.
        Returns a dict with normalized coords, pixel coords, z, visibility.
        """
        lm = pose_landmarks.landmark
        P = self.mp_pose.PoseLandmark

        target_names = {
            "nose": P.NOSE,
            "left_shoulder": P.LEFT_SHOULDER,
            "right_shoulder": P.RIGHT_SHOULDER,
            "left_elbow": P.LEFT_ELBOW,
            "right_elbow": P.RIGHT_ELBOW,
            "left_wrist": P.LEFT_WRIST,
            "right_wrist": P.RIGHT_WRIST,
            "left_hip": P.LEFT_HIP,
            "right_hip": P.RIGHT_HIP,
        }

        result = {}
        for name, idx in target_names.items():
            pt = lm[idx]
            px = int(pt.x * img_w)
            py = int(pt.y * img_h)
            result[name] = {
                "x_norm": float(pt.x),
                "y_norm": float(pt.y),
                "z_norm": float(pt.z),
                "visibility": float(pt.visibility),
                "x_px": px,
                "y_px": py,
            }

        # Optional derived points
        ls = result["left_shoulder"]
        rs = result["right_shoulder"]
        lh = result["left_hip"]
        rh = result["right_hip"]

        result["shoulder_center"] = {
            "x_norm": 0.5 * (ls["x_norm"] + rs["x_norm"]),
            "y_norm": 0.5 * (ls["y_norm"] + rs["y_norm"]),
            "z_norm": 0.5 * (ls["z_norm"] + rs["z_norm"]),
            "visibility": 0.5 * (ls["visibility"] + rs["visibility"]),
            "x_px": int(0.5 * (ls["x_px"] + rs["x_px"])),
            "y_px": int(0.5 * (ls["y_px"] + rs["y_px"])),
        }

        result["hip_center"] = {
            "x_norm": 0.5 * (lh["x_norm"] + rh["x_norm"]),
            "y_norm": 0.5 * (lh["y_norm"] + rh["y_norm"]),
            "z_norm": 0.5 * (lh["z_norm"] + rh["z_norm"]),
            "visibility": 0.5 * (lh["visibility"] + rh["visibility"]),
            "x_px": int(0.5 * (lh["x_px"] + rh["x_px"])),
            "y_px": int(0.5 * (lh["y_px"] + rh["y_px"])),
        }

        return result

    def draw_upper_body_pose(self, image, pose_landmarks):
        """
        Draw only upper-body pose connections and keypoints.
        """
        lm = pose_landmarks.landmark
        P = self.mp_pose.PoseLandmark

        upper_body_connections = [
            (P.NOSE, P.LEFT_SHOULDER),
            (P.NOSE, P.RIGHT_SHOULDER),
            (P.LEFT_SHOULDER, P.RIGHT_SHOULDER),
            (P.LEFT_SHOULDER, P.LEFT_ELBOW),
            (P.LEFT_ELBOW, P.LEFT_WRIST),
            (P.RIGHT_SHOULDER, P.RIGHT_ELBOW),
            (P.RIGHT_ELBOW, P.RIGHT_WRIST),
            (P.LEFT_SHOULDER, P.LEFT_HIP),
            (P.RIGHT_SHOULDER, P.RIGHT_HIP),
            (P.LEFT_HIP, P.RIGHT_HIP),
        ]

        h, w = image.shape[:2]

        # draw connections
        for a, b in upper_body_connections:
            pa = lm[a]
            pb = lm[b]

            if pa.visibility > 0.5 and pb.visibility > 0.5:
                xa, ya = int(pa.x * w), int(pa.y * h)
                xb, yb = int(pb.x * w), int(pb.y * h)
                cv2.line(image, (xa, ya), (xb, yb), (255, 255, 0), 2)

        # draw selected landmarks
        selected_points = [
            P.NOSE,
            P.LEFT_SHOULDER, P.RIGHT_SHOULDER,
            P.LEFT_ELBOW, P.RIGHT_ELBOW,
            P.LEFT_WRIST, P.RIGHT_WRIST,
            P.LEFT_HIP, P.RIGHT_HIP
        ]

        for idx in selected_points:
            pt = lm[idx]
            if pt.visibility > 0.5:
                x, y = int(pt.x * w), int(pt.y * h)
                cv2.circle(image, (x, y), 5, (0, 255, 255), -1)

    def camera_loop(self):
        pad_rect = (120, 150, 520, 350)
        prev_time = time.time()

        while self.is_running and rclpy.ok():
            curr_time = time.time()
            if curr_time - prev_time < 0.05:
                time.sleep(0.01)
                continue
            prev_time = curr_time

            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            small_img = cv2.resize(image, (320, 240))
            rgb_small = cv2.cvtColor(small_img, cv2.COLOR_BGR2RGB)

            # Process both Hands and Pose on the same resized frame
            hand_results = self.hands.process(rgb_small)
            pose_results = self.pose.process(rgb_small)

            gestures = {"Left": "None", "Right": "None"}

            # -----------------------------
            # Pose processing (newly added)
            # -----------------------------
            if pose_results.pose_landmarks:
                upper_body_data = self.extract_upper_body_landmarks(
                    pose_results.pose_landmarks,
                    img_w=640,
                    img_h=480
                )
                self.upper_body_pub.publish(String(data=json.dumps(upper_body_data)))
                self.draw_upper_body_pose(image, pose_results.pose_landmarks)

                # Optional on-screen labels
                for name in ["left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist"]:
                    px = upper_body_data[name]["x_px"]
                    py = upper_body_data[name]["y_px"]
                    vis = upper_body_data[name]["visibility"]
                    if vis > 0.5 and 0 <= px < 640 and 0 <= py < 480:
                        cv2.putText(
                            image,
                            name.replace("_", " "),
                            (px + 5, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.35,
                            (255, 255, 0),
                            1
                        )

            # -----------------------------
            # Hands processing (original)
            # -----------------------------
            if hand_results.multi_hand_landmarks:
                for idx, hand_landmarks in enumerate(hand_results.multi_hand_landmarks):
                    label = hand_results.multi_handedness[idx].classification[0].label
                    gesture = self.recognize_gesture(hand_landmarks, label)

                    # Keep original mirrored assignment logic exactly
                    if label == "Right":
                        gestures["Left"] = gesture
                        if gesture == "pointing" and self.last_left_gesture != "pointing" and self.robot_state == "NOT READY":
                            self.current_idx = (self.current_idx + 1) % len(self.surfaces)
                        self.last_left_gesture = gesture

                    elif label == "Left":
                        gestures["Right"] = gesture
                        cx = int(hand_landmarks.landmark[8].x * 640)
                        cy = int(hand_landmarks.landmark[8].y * 480)

                        if self.robot_state == "FOLLOWING":
                            if 0 <= cx < 640 and 0 <= cy < 480:
                                d_list = []
                                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)]:
                                    sx = cx + dx
                                    sy = cy + dy
                                    if 0 <= sx < 640 and 0 <= sy < 480:
                                        val = depth_frame.get_distance(sx, sy)
                                        if val > 0.05:
                                            d_list.append(val)

                                if len(d_list) > 0:
                                    d = np.median(d_list)
                                    p3d = rs.rs2_deproject_pixel_to_point(self.intr, [cx, cy], d)
                                    self.target_pub.publish(Point(
                                        x=float(p3d[0]),
                                        y=float(p3d[1]),
                                        z=float(p3d[2])
                                    ))
                                    cv2.circle(image, (cx, cy), 15, (0, 255, 0), -1)

                        elif self.robot_state == "GUIDED_FOLLOWING":
                            cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
                            px = np.clip((cx - pad_rect[0]) / (pad_rect[2] - pad_rect[0]), 0.0, 1.0)
                            py = np.clip((cy - pad_rect[1]) / (pad_rect[3] - pad_rect[1]), 0.0, 1.0)
                            self.guided_pub.publish(Point(x=float(px), y=float(py), z=0.0))

                    self.mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )

            self.gesture_pub.publish(String(data=f"Left:{gestures['Left']},Right:{gestures['Right']}"))

            if gestures["Left"] == "okay" and gestures["Right"] == "okay":
                if not self.launch_triggered and self.robot_state == "NOT READY":
                    msg = String()
                    msg.data = f"LAUNCH:{self.surfaces[self.current_idx]}"
                    self.surface_cmd_pub.publish(msg)
                    self.launch_triggered = True
            elif self.robot_state != "NOT READY":
                self.launch_triggered = True
            else:
                self.launch_triggered = False

            display = cv2.flip(image, 1)

            cv2.rectangle(display, (0, 0), (640, 100), (0, 0, 0), -1)
            status_color = (
                (0, 255, 0) if self.robot_state == "FOLLOWING"
                else (255, 100, 255) if self.robot_state == "GUIDED_FOLLOWING"
                else (0, 165, 255) if self.robot_state != "ESTOP"
                else (0, 0, 255)
            )

            cv2.putText(display, f"ROBOT: {self.robot_state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv2.putText(display, f"FPS: {1.0 / (time.time() - curr_time + 0.001):.1f}", (540, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if self.robot_state == "NOT READY":
                cv2.putText(display, f"R-POINT to CHANGE: {self.surfaces[self.current_idx]}", (10, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            flipped_pad = (640 - pad_rect[2], pad_rect[1], 640 - pad_rect[0], pad_rect[3])
            if self.robot_state == "GUIDED_FOLLOWING":
                cv2.rectangle(display, (flipped_pad[0], flipped_pad[1]),
                              (flipped_pad[2], flipped_pad[3]), (255, 0, 255), 2)
                cv2.putText(display, f"PAD: {self.surfaces[self.current_idx]}",
                            (flipped_pad[0], flipped_pad[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            # Additional info line for pose status
            pose_status = "ON" if pose_results.pose_landmarks else "OFF"
            cv2.putText(display, f"UPPER BODY POSE: {pose_status}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.imshow('Vision V7', display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def destroy_node(self):
        self.is_running = False
        self.vision_thread.join()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        self.hands.close()
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedVisionNodeLightV7()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()