import sys
import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState  # [추가] 몸체 제어용 메시지 타입
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
        
        # [추가] 모빌리티(/cmd_vel) 및 몸체(/forward_aux_joint_targets) 퍼블리셔 등록
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
        self.baseline_ears = []
        self.calibrated_eyebrow_ratio = 0.10 
        self.calibrated_ear = 0.25 
        
        self.current_state = "IDLE"
        self.right_fist_timer = 0.0
        self.gesture_timers = {
            "TURN_LEFT": 0.0, "TURN_RIGHT": 0.0,
            "EYEBROW_RAISE": 0.0, "WINK_LEFT": 0.0, "WINK_RIGHT": 0.0
        }
        
        self.last_gesture_time = time.time()
        self.gesture_cooldown = 1.0  
        self.active_gesture_msg = "State: IDLE (Ready for Gesture)"
        self.msg_clear_time = 0.0    

        # [추가] 실시간 전진/정지 상태 추적 플래그
        self.robot_moving = False

        # 각 커맨드별 손 관절 데이터 정의 (기존 인터페이스 유지용)
        self.CMD_1_OPEN = [0.0] * 40
        self.CMD_6_L_CLOSE_R_OPEN = [0.2, 0.3, 0.3, 0.3,  0.3, 0.5, 0.5, 0.5,  0.3, 0.5, 0.5, 0.5,  0.3, 0.5, 0.5, 0.5,  0.3, 0.5, 0.5, 0.5,  
                                     0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0]
        self.CMD_7_L_OPEN_R_CLOSE = [0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,  
                                     0.2, 0.3, 0.3, 0.3,  0.3, 0.5, 0.5, 0.5,  0.3, 0.5, 0.5, 0.5,  0.3, 0.5, 0.5, 0.5,  0.3, 0.5, 0.5, 0.5]

        # [추가] 몸체 관절 공통 및 고유 데이터 정의 (sensor_msgs/msg/JointState 매핑용)
        self.joint_names = ['torso_0', 'torso_1', 'torso_2', 'torso_3', 'torso_4', 'torso_5']
        self.joint_velocities = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
        
        self.pos_turn_left = [0.0, 0.0875, 0.0883, -0.1739, 0.0, 1.5708]
        self.pos_turn_right = [0.0, 0.0875, 0.0883, -0.1739, 0.0, -1.5708]
        self.pos_center = [0.0, 0.0875, 0.0883, -0.1739, 0.0, 0.0]

        self.vision_thread = threading.Thread(target=self.camera_loop)
        self.vision_thread.start()

    # 모빌리티(/cmd_vel) 속도 전송 함수
    def send_speed_cmd(self, linear_x, angular_z=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    # 몸체 관절(/forward_aux_joint_targets) 명령 전송 함수
    def send_aux_joint_cmd(self, positions):
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = positions
        joint_state.velocity = self.joint_velocities
        self.aux_joint_pub.publish(joint_state)

    def get_eye_aspect_ratio(self, landmarks, top, bottom, left, right):
        v_dist = np.linalg.norm(np.array([landmarks[top].x, landmarks[top].y]) - 
                                np.array([landmarks[bottom].x, landmarks[bottom].y]))
        h_dist = np.linalg.norm(np.array([landmarks[left].x, landmarks[left].y]) - 
                                np.array([landmarks[right].x, landmarks[right].y]))
        return v_dist / h_dist if h_dist > 0 else 0

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
                    left_ear = self.get_eye_aspect_ratio(lm, 159, 145, 33, 133); right_ear = self.get_eye_aspect_ratio(lm, 386, 374, 362, 263)
                    avg_ear = (left_ear + right_ear) / 2.0

                    if self.calibration_start_time is None: self.calibration_start_time = current_time
                    if self.is_calibrating:
                        elapsed_calib = current_time - self.calibration_start_time
                        if elapsed_calib < self.calibration_duration:
                            self.baseline_eyebrow_ratios.append(eyebrow_ratio); self.baseline_ears.append(avg_ear) 
                            cv2.putText(display, f"CALIBRATION: {self.calibration_duration - elapsed_calib:.1f}s", (180, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
                            continue 
                        else:
                            self.is_calibrating = False
                            if self.baseline_eyebrow_ratios and self.baseline_ears:
                                self.calibrated_eyebrow_ratio = sum(self.baseline_eyebrow_ratios) / len(self.baseline_eyebrow_ratios)
                                self.calibrated_ear = sum(self.baseline_ears) / len(self.baseline_ears)

                    dx, dz = lm[454].x - lm[234].x, lm[454].z - lm[234].z
                    yaw_angle = math.degrees(math.atan2(dz, dx)) 
                    is_mouth_open = (abs(lm[13].y - lm[14].y) / face_height) > 0.05 
                    is_eyebrow_raise = eyebrow_ratio > (self.calibrated_eyebrow_ratio * 1.15)
                    is_wink_left = left_ear < (self.calibrated_ear * 0.65) and right_ear > (self.calibrated_ear * 0.75)
                    is_wink_right = right_ear < (self.calibrated_ear * 0.65) and left_ear > (self.calibrated_ear * 0.75)

                    # [기능 1] 입 벌리기 실시간 제어 구문 (v2 로직)
                    if is_mouth_open:
                        self.send_speed_cmd(linear_x=0.3, angular_z=0.0)
                        self.robot_moving = True
                        if self.current_state == "IDLE":
                            self.active_gesture_msg = "Mouth Open : Robot Moving Forward"
                    else:
                        if self.robot_moving:
                            self.send_speed_cmd(linear_x=0.0, angular_z=0.0)
                            self.robot_moving = False
                            if self.current_state == "IDLE":
                                self.active_gesture_msg = "Mouth Closed : Robot Stopped"

                    def check_gesture(name, condition, duration_req):
                        if condition:
                            if self.gesture_timers[name] == 0.0: self.gesture_timers[name] = current_time
                            elapsed = current_time - self.gesture_timers[name]
                            return elapsed >= duration_req, elapsed
                        self.gesture_timers[name] = 0.0; return False, 0.0

                    # v1 기반 인터페이스 제어 구문 (타이머 판단 및 잠금)
                    if self.current_state == "IDLE":
                        t_left_trig, t_left_el = check_gesture("TURN_LEFT", yaw_angle < -25.0, 3.0)
                        t_right_trig, t_right_el = check_gesture("TURN_RIGHT", yaw_angle > 25.0, 3.0)
                        brow_trig, brow_el = check_gesture("EYEBROW_RAISE", is_eyebrow_raise, 2.0)
                        wink_l_trig, wink_l_el = check_gesture("WINK_LEFT", is_wink_left, 2.0)
                        wink_r_trig, wink_r_el = check_gesture("WINK_RIGHT", is_wink_right, 2.0)

                        triggered_gesture = None
                        if t_left_trig: triggered_gesture = "TURN_LEFT"
                        elif t_right_trig: triggered_gesture = "TURN_RIGHT"
                        elif brow_trig: triggered_gesture = "EYEBROW_RAISE"
                        elif wink_l_trig: triggered_gesture = "WINK_LEFT"
                        elif wink_r_trig: triggered_gesture = "WINK_RIGHT"

                        if triggered_gesture:
                            self.current_state = triggered_gesture
                            self.msg_clear_time = current_time + 2.0 
                            self.gesture_pub.publish(String(data=triggered_gesture))

                            # [기능 2, 3, 4] 및 기존 윙크 로봇 손 퍼블리시 매핑 수정
                            if triggered_gesture == "TURN_LEFT":
                                self.send_aux_joint_cmd(self.pos_turn_left)
                                self.active_gesture_msg = "Turn Left : Rotate waist left"
                            elif triggered_gesture == "TURN_RIGHT":
                                self.send_aux_joint_cmd(self.pos_turn_right)
                                self.active_gesture_msg = "Turn Right : Rotate waist right"
                            elif triggered_gesture == "EYEBROW_RAISE":
                                self.send_aux_joint_cmd(self.pos_center)
                                self.active_gesture_msg = "Eyebrow Raise : Return waist to center"
                            elif triggered_gesture == "WINK_LEFT": 
                                self.joint_pub.publish(Float64MultiArray(data=self.CMD_6_L_CLOSE_R_OPEN))
                                self.active_gesture_msg = "Wink Left : Left hand close, Right hand open"
                            elif triggered_gesture == "WINK_RIGHT": 
                                self.joint_pub.publish(Float64MultiArray(data=self.CMD_7_L_OPEN_R_CLOSE))
                                self.active_gesture_msg = "Wink Right : Left hand open, Right hand close"

                            self.gesture_timers[triggered_gesture] = 0.0 
                    else:
                        for k in self.gesture_timers: self.gesture_timers[k] = 0.0
                        t_left_el = t_right_el = brow_el = wink_l_el = wink_r_el = 0.0

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
                                
                                # 손 초기화 발행
                                self.joint_pub.publish(Float64MultiArray(data=self.CMD_1_OPEN))
                                self.right_fist_timer = 0.0
                        else: self.right_fist_timer = 0.0

                    # 인터페이스 텍스트 출력 로직 정리
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
                        elif wink_l_el > 0: display_text = f"Winking Left... {wink_l_el:.1f}s / 2.0s"
                        elif wink_r_el > 0: display_text = f"Winking Right... {wink_r_el:.1f}s / 2.0s"
                        else: 
                            display_text = self.active_gesture_msg
                            text_color = (0, 255, 0) if not is_mouth_open else (0, 0, 255)
                        
                    cv2.putText(display, display_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
                    if not self.is_calibrating: cv2.putText(display, f"Yaw: {yaw_angle:.1f}", (520, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            elapsed = time.time() - start_time; fps = 1.0 / elapsed if elapsed > 0 else 30.0
            cv2.putText(display, f"FPS: {fps:.1f} | Hand & Face Control v3", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.imshow('Ultra Light Tracker', display)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            time.sleep(max(0, (1.0 / 30.0) - (time.time() - start_time)))

    def destroy_node(self):
        # 안전장치: 노드 종료 시 모빌리티 정지 명령
        self.send_speed_cmd(0.0, 0.0)
        self.is_running = False; self.vision_thread.join(); self.pipeline.stop(); cv2.destroyAllWindows(); super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = UltraLightMultiTracker()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__': main()
