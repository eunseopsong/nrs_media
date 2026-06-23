import sys
import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
import threading
import time
import math

class UltraLightMultiTracker(Node):
    def __init__(self):
        super().__init__('ultra_light_tracker')
        self.left_pub = self.create_publisher(Point, '/left_hand', 10)
        self.right_pub = self.create_publisher(Point, '/right_hand', 10)
        self.face_pub = self.create_publisher(Point, '/face_pose', 10)
        self.gesture_pub = self.create_publisher(String, '/face_gesture_cmd', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/forward_hand_joint_targets', 10)
        
        # 모빌리티(/cmd_vel) 및 몸체(/forward_aux_joint_targets) 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.aux_joint_pub = self.create_publisher(JointState, '/forward_aux_joint_targets', 10)

        self.mp_hands = mp.solutions.hands
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.hands = self.mp_hands.Hands(max_num_hands=2, model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.face_mesh = self.mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)

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

        self.size_to_depth_c = {"Left": 40.0, "Right": 40.0} 
        self.is_running = True
        
        self.is_calibrating = True
        self.calibration_duration = 5.0
        self.calibration_start_time = None
        self.baseline_eyebrow_ratios = []
        self.calibrated_eyebrow_ratio = 0.10 
        
        self.current_state = "IDLE"
        self.right_fist_timer = 0.0
        
        # Wink 제거됨. 고개 돌리기 및 눈썹 올리기만 유지 (v3 기준)
        self.gesture_timers = {
            "TURN_LEFT": 0.0, "TURN_RIGHT": 0.0, "EYEBROW_RAISE": 0.0
        }
        
        self.last_gesture_time = time.time()
        self.gesture_cooldown = 1.0  
        self.active_gesture_msg = "State: IDLE (Ready for Gesture)"
        self.msg_clear_time = 0.0    

        self.CMD_1_OPEN = [0.0] * 40

        self.joint_names = ['torso_0', 'torso_1', 'torso_2', 'torso_3', 'torso_4', 'torso_5']
        self.joint_velocities = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        self.pos_turn_left = [0.0, 0.0875, 0.0883, -0.1739, 0.0, 1.5708]
        self.pos_turn_right = [0.0, 0.0875, 0.0883, -0.1739, 0.0, -1.5708]
        self.pos_center = [0.0, 0.0875, 0.0883, -0.1739, 0.0, 0.0]

        self.vision_thread = threading.Thread(target=self.camera_loop)
        self.vision_thread.start()

    def send_speed_cmd(self, linear_x, angular_z=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def send_aux_joint_cmd(self, positions):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = positions
        joint_state.velocity = self.joint_velocities
        self.aux_joint_pub.publish(joint_state)

    def is_fist(self, landmarks):
        wrist = np.array([landmarks[0].x, landmarks[0].y, landmarks[0].z])
        fingertips = [8, 12, 16, 20]; pips = [6, 10, 14, 18]
        for tip, pip in zip(fingertips, pips):
            tip_pos = np.array([landmarks[tip].x, landmarks[tip].y, landmarks[tip].z])
            pip_pos = np.array([landmarks[pip].x, landmarks[pip].y, landmarks[pip].z])
            if np.linalg.norm(tip_pos - wrist) > np.linalg.norm(pip_pos - wrist): return False
        return True

    def camera_loop(self):
        while self.is_running and rclpy.ok():
            start_time = time.time()
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame, depth_frame = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color_frame or not depth_frame: continue

            image = np.asanyarray(color_frame.get_data())
            display = cv2.flip(image, 1); rgb_display = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            hand_results = self.hands.process(rgb_display); face_results = self.face_mesh.process(rgb_display)
            current_time = time.time()

            is_right_fist_active = False
            if hand_results.multi_hand_landmarks:
                for idx, hand_landmarks in enumerate(hand_results.multi_hand_landmarks):
                    label = hand_results.multi_handedness[idx].classification[0].label 
                    if label == "Right" and self.is_fist(hand_landmarks.landmark): is_right_fist_active = True
                    cx, cy = int(hand_landmarks.landmark[9].x * 640), int(hand_landmarks.landmark[9].y * 480)
                    wx, wy = int(hand_landmarks.landmark[0].x * 640), int(hand_landmarks.landmark[0].y * 480)
                    if 20 < cx < 620 and 20 < cy < 460: 
                        hand_len_px = max(5.0, np.sqrt((cx - wx)**2 + (cy - wy)**2))
                        depth_x = min(639, max(0, 639 - cx)) 
                        raw_dist = depth_frame.get_distance(depth_x, cy)
                        if 0.35 < raw_dist < 1.2 and hand_len_px > 40:
                            final_dist = raw_dist
                            self.size_to_depth_c[label] = 0.9 * self.size_to_depth_c[label] + 0.1 * (raw_dist * hand_len_px)
                        else: final_dist = self.size_to_depth_c[label] / hand_len_px
                        if 0.05 < final_dist < 2.0: 
                            p3d = rs.rs2_deproject_pixel_to_point(self.intr, [depth_x, cy], final_dist)
                            msg = Point(x=float(p3d[0]), y=float(p3d[1]), z=float(p3d[2]))
                            if label == "Left": self.left_pub.publish(msg)
                            else: self.right_pub.publish(msg)
                        self.mp_drawing.draw_landmarks(display, hand_landmarks, self.mp_hands.HAND_CONNECTIONS, self.mp_drawing_styles.get_default_hand_landmarks_style(), self.mp_drawing_styles.get_default_hand_connections_style())

            if face_results.multi_face_landmarks:
                for face_landmarks in face_results.multi_face_landmarks:
                    self.mp_drawing.draw_landmarks(display, face_landmarks, self.mp_face_mesh.FACEMESH_TESSELATION, None, self.mp_drawing_styles.get_default_face_mesh_tesselation_style())
                    lm = face_landmarks.landmark
                    left_eyebrow_raise = abs(lm[159].y - lm[105].y); right_eyebrow_raise = abs(lm[386].y - lm[334].y)
                    face_height = abs(lm[10].y - lm[152].y) + 1e-6
                    eyebrow_ratio = ((left_eyebrow_raise + right_eyebrow_raise) / 2.0) / face_height

                    if self.calibration_start_time is None: self.calibration_start_time = current_time
                    if self.is_calibrating:
                        elapsed_calib = current_time - self.calibration_start_time
                        if elapsed_calib < self.calibration_duration:
                            self.baseline_eyebrow_ratios.append(eyebrow_ratio)
                            cv2.putText(display, f"CALIBRATION: {self.calibration_duration - elapsed_calib:.1f}s", (180, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
                            continue 
                        else:
                            self.is_calibrating = False
                            if self.baseline_eyebrow_ratios:
                                self.calibrated_eyebrow_ratio = sum(self.baseline_eyebrow_ratios) / len(self.baseline_eyebrow_ratios)

                    dx, dz = lm[454].x - lm[234].x, lm[454].z - lm[234].z
                    yaw_angle = math.degrees(math.atan2(dz, dx)) 
                    is_mouth_open = (abs(lm[13].y - lm[14].y) / face_height) > 0.05 
                    is_eyebrow_raise = eyebrow_ratio > (self.calibrated_eyebrow_ratio * 1.15)

                    # ---------------------------------------------------------
                    # [핵심 수정] 아이작 심 대응 -r 20 (연속 발행) 로직
                    # ---------------------------------------------------------
                    # 1회성 발행 플래그를 버리고, 루프가 도는 매 프레임마다(약 30Hz)
                    # 현재 입 상태에 맞는 속도를 무조건 지속적으로 퍼블리시합니다.
                    
                    if is_mouth_open:
                        current_speed = 0.3
                        if self.current_state == "IDLE":
                            self.active_gesture_msg = "Mouth Open : Robot Moving Forward"
                    else:
                        current_speed = 0.0
                        if self.current_state == "IDLE":
                            self.active_gesture_msg = "Mouth Closed : Robot Stopped"
                            
                    # 매 프레임마다 끊임없이 0.3 또는 0.0을 보냅니다 (터미널의 -r 20과 동일한 효과)
                    self.send_speed_cmd(linear_x=current_speed, angular_z=0.0)
                    # ---------------------------------------------------------

                    def check_gesture(name, condition, duration_req):
                        if condition:
                            if self.gesture_timers[name] == 0.0: self.gesture_timers[name] = current_time
                            elapsed = current_time - self.gesture_timers[name]
                            return elapsed >= duration_req, elapsed
                        self.gesture_timers[name] = 0.0; return False, 0.0

                    # 안면 제스처 (Wink 삭제됨)
                    if self.current_state == "IDLE":
                        t_left_trig, t_left_el = check_gesture("TURN_LEFT", yaw_angle < -25.0, 3.0)
                        t_right_trig, t_right_el = check_gesture("TURN_RIGHT", yaw_angle > 25.0, 3.0)
                        brow_trig, brow_el = check_gesture("EYEBROW_RAISE", is_eyebrow_raise, 2.0)

                        triggered_gesture = None
                        if t_left_trig: triggered_gesture = "TURN_LEFT"
                        elif t_right_trig: triggered_gesture = "TURN_RIGHT"
                        elif brow_trig: triggered_gesture = "EYEBROW_RAISE"

                        if triggered_gesture:
                            self.current_state = triggered_gesture
                            self.msg_clear_time = current_time + 2.0 
                            self.gesture_pub.publish(String(data=triggered_gesture))

                            if triggered_gesture == "TURN_LEFT":
                                self.send_aux_joint_cmd(self.pos_turn_left)
                                self.active_gesture_msg = "Turn Left : Rotate waist left"
                            elif triggered_gesture == "TURN_RIGHT":
                                self.send_aux_joint_cmd(self.pos_turn_right)
                                self.active_gesture_msg = "Turn Right : Rotate waist right"
                            elif triggered_gesture == "EYEBROW_RAISE":
                                self.send_aux_joint_cmd(self.pos_center)
                                self.active_gesture_msg = "Eyebrow Raise : Return waist to center"

                            self.gesture_timers[triggered_gesture] = 0.0 
                    else:
                        for k in self.gesture_timers: self.gesture_timers[k] = 0.0
                        t_left_el = t_right_el = brow_el = 0.0

                    # V3 기준 리셋 제스처 (오른손 주먹 3초 유지)
                    fist_elapsed = 0.0
                    if self.current_state != "IDLE":
                        if is_right_fist_active:
                            if self.right_fist_timer == 0.0: self.right_fist_timer = current_time
                            fist_elapsed = current_time - self.right_fist_timer
                            if fist_elapsed >= 3.0:
                                self.current_state = "IDLE"
                                self.active_gesture_msg = "Right Fist : Reset hand joints to open"
                                self.msg_clear_time = current_time + 2.0
                                self.gesture_pub.publish(String(data="IDLE"))
                                
                                self.joint_pub.publish(Float64MultiArray(data=self.CMD_1_OPEN))
                                self.right_fist_timer = 0.0
                        else: self.right_fist_timer = 0.0

                    # 인터페이스 텍스트 렌더링
                    text_color = (0, 165, 255)
                    if current_time < self.msg_clear_time: 
                        display_text = self.active_gesture_msg
                        text_color = (0, 0, 255) 
                    elif self.current_state != "IDLE":
                        if is_right_fist_active: 
                            display_text = f"Resetting to IDLE... {fist_elapsed:.1f}s / 3.0s"
                            text_color = (0, 255, 255)
                        else: 
                            display_text = f"LOCKED: {self.current_state} (Right Fist 3s to Reset)"
                            text_color = (0, 0, 255)
                    else:
                        if t_left_el > 0: display_text = f"Turning Left... {t_left_el:.1f}s / 3.0s"
                        elif t_right_el > 0: display_text = f"Turning Right... {t_right_el:.1f}s / 3.0s"
                        elif brow_el > 0: display_text = f"Raising Brows... {brow_el:.1f}s / 2.0s"
                        else: 
                            display_text = self.active_gesture_msg
                            text_color = (0, 255, 0) if not is_mouth_open else (0, 0, 255)
                        
                    cv2.putText(display, display_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
                    if not self.is_calibrating: cv2.putText(display, f"Yaw: {yaw_angle:.1f}", (520, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            elapsed = time.time() - start_time; fps = 1.0 / elapsed if elapsed > 0 else 30.0
            cv2.putText(display, f"FPS: {fps:.1f} | Hand & Face Control v6", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.imshow('Ultra Light Tracker', display)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            time.sleep(max(0, (1.0 / 30.0) - (time.time() - start_time)))

    def destroy_node(self):
        # 종료 시 확실하게 멈추도록 마지막으로 0.0 전송
        self.send_speed_cmd(0.0, 0.0)
        self.is_running = False; self.vision_thread.join(); self.pipeline.stop(); cv2.destroyAllWindows(); super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = UltraLightMultiTracker()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__': main()
