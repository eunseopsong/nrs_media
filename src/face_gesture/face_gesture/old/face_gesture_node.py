import sys
import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
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
        
        # 캘리브레이션 변수
        self.is_calibrating = True
        self.calibration_duration = 5.0
        self.calibration_start_time = None
        self.baseline_eyebrow_ratios = []
        self.baseline_ears = []
        self.calibrated_eyebrow_ratio = 0.10 
        self.calibrated_ear = 0.25 
        
        # [추가] 상태 머신 (State Machine) 변수
        self.current_state = "IDLE"
        self.right_fist_timer = 0.0
        
        self.gesture_timers = {
            "TURN_LEFT": 0.0,
            "TURN_RIGHT": 0.0,
            "MOUTH_OPEN": 0.0,
            "EYEBROW_RAISE": 0.0,
            "WINK_LEFT": 0.0,
            "WINK_RIGHT": 0.0
        }
        
        self.last_gesture_time = time.time()
        self.gesture_cooldown = 1.0  
        self.active_gesture_msg = "None"
        self.msg_clear_time = 0.0    

        self.vision_thread = threading.Thread(target=self.camera_loop)
        self.vision_thread.start()

    def get_eye_aspect_ratio(self, landmarks, top, bottom, left, right):
        v_dist = np.linalg.norm(np.array([landmarks[top].x, landmarks[top].y]) - 
                                np.array([landmarks[bottom].x, landmarks[bottom].y]))
        h_dist = np.linalg.norm(np.array([landmarks[left].x, landmarks[left].y]) - 
                                np.array([landmarks[right].x, landmarks[right].y]))
        return v_dist / h_dist if h_dist > 0 else 0

    # [추가] 3D 좌표 기반 주먹(Fist) 인식 함수
    def is_fist(self, landmarks):
        wrist = np.array([landmarks[0].x, landmarks[0].y, landmarks[0].z])
        fingertips = [8, 12, 16, 20] # 검지, 중지, 약지, 소지 끝
        pips = [6, 10, 14, 18]       # 두 번째 마디
        
        for tip, pip in zip(fingertips, pips):
            tip_pos = np.array([landmarks[tip].x, landmarks[tip].y, landmarks[tip].z])
            pip_pos = np.array([landmarks[pip].x, landmarks[pip].y, landmarks[pip].z])
            # 손가락 끝이 두 번째 마디보다 손목에 멀리 있다면 펴진 것으로 간주
            if np.linalg.norm(tip_pos - wrist) > np.linalg.norm(pip_pos - wrist):
                return False
        return True

    def camera_loop(self):
        while self.is_running and rclpy.ok():
            start_time = time.time()
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame, depth_frame = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color_frame or not depth_frame: continue

            image = np.asanyarray(color_frame.get_data())
            display = cv2.flip(image, 1) 
            rgb_display = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            
            hand_results = self.hands.process(rgb_display)
            face_results = self.face_mesh.process(rgb_display)
            
            current_time = time.time()

            # ---------------------------------------------------------
            # 1. 손 인식 및 주먹 상태 확인 (Hands Processing)
            # ---------------------------------------------------------
            is_right_fist_active = False
            
            if hand_results.multi_hand_landmarks:
                for idx, hand_landmarks in enumerate(hand_results.multi_hand_landmarks):
                    label = hand_results.multi_handedness[idx].classification[0].label 
                    
                    # 오른손 주먹 여부 판단
                    if label == "Right" and self.is_fist(hand_landmarks.landmark):
                        is_right_fist_active = True
                    
                    cx, cy = int(hand_landmarks.landmark[9].x * 640), int(hand_landmarks.landmark[9].y * 480)
                    wx, wy = int(hand_landmarks.landmark[0].x * 640), int(hand_landmarks.landmark[0].y * 480)

                    if 20 < cx < 620 and 20 < cy < 460: 
                        hand_len_px = max(5.0, np.sqrt((cx - wx)**2 + (cy - wy)**2))
                        depth_x = min(639, max(0, 639 - cx)) 
                        raw_dist = depth_frame.get_distance(depth_x, cy)
                        
                        if 0.35 < raw_dist < 1.2 and hand_len_px > 40:
                            final_dist = raw_dist
                            self.size_to_depth_c[label] = 0.9 * self.size_to_depth_c[label] + 0.1 * (raw_dist * hand_len_px)
                        else:
                            final_dist = self.size_to_depth_c[label] / hand_len_px

                        if 0.05 < final_dist < 2.0: 
                            p3d = rs.rs2_deproject_pixel_to_point(self.intr, [depth_x, cy], final_dist)
                            msg = Point(x=float(p3d[0]), y=float(p3d[1]), z=float(p3d[2]))
                            if label == "Left": self.left_pub.publish(msg)
                            else: self.right_pub.publish(msg)
                        
                        self.mp_drawing.draw_landmarks(display, hand_landmarks, self.mp_hands.HAND_CONNECTIONS, self.mp_drawing_styles.get_default_hand_landmarks_style(), self.mp_drawing_styles.get_default_hand_connections_style())

            # ---------------------------------------------------------
            # 2. 얼굴 인식 및 캘리브레이션 (Face Processing)
            # ---------------------------------------------------------
            yaw_angle = 0.0
            
            if face_results.multi_face_landmarks:
                for face_landmarks in face_results.multi_face_landmarks:
                    
                    self.mp_drawing.draw_landmarks(display, face_landmarks, self.mp_face_mesh.FACEMESH_TESSELATION, None, self.mp_drawing_styles.get_default_face_mesh_tesselation_style())
                    self.mp_drawing.draw_landmarks(display, face_landmarks, self.mp_face_mesh.FACEMESH_CONTOURS, None, self.mp_drawing_styles.get_default_face_mesh_contours_style())
                    self.mp_drawing.draw_landmarks(display, face_landmarks, self.mp_face_mesh.FACEMESH_IRISES, None, self.mp_drawing_styles.get_default_face_mesh_iris_connections_style())

                    nx, ny = int(face_landmarks.landmark[1].x * 640), int(face_landmarks.landmark[1].y * 480)
                    if 20 < nx < 620 and 20 < ny < 460:
                        depth_nx = min(639, max(0, 639 - nx))
                        face_dist = depth_frame.get_distance(depth_nx, ny)
                        if 0.2 < face_dist < 2.5:
                            p3d = rs.rs2_deproject_pixel_to_point(self.intr, [depth_nx, ny], face_dist)
                            self.face_pub.publish(Point(x=float(p3d[0]), y=float(p3d[1]), z=float(p3d[2])))

                    lm = face_landmarks.landmark
                    
                    left_eyebrow_raise = abs(lm[159].y - lm[105].y)
                    right_eyebrow_raise = abs(lm[386].y - lm[334].y)
                    face_height = abs(lm[10].y - lm[152].y) + 1e-6
                    eyebrow_ratio = ((left_eyebrow_raise + right_eyebrow_raise) / 2.0) / face_height

                    left_ear = self.get_eye_aspect_ratio(lm, top=159, bottom=145, left=33, right=133)
                    right_ear = self.get_eye_aspect_ratio(lm, top=386, bottom=374, left=362, right=263)
                    avg_ear = (left_ear + right_ear) / 2.0

                    if self.calibration_start_time is None:
                        self.calibration_start_time = current_time
                        
                    if self.is_calibrating:
                        elapsed_calib = current_time - self.calibration_start_time
                        if elapsed_calib < self.calibration_duration:
                            self.baseline_eyebrow_ratios.append(eyebrow_ratio)
                            self.baseline_ears.append(avg_ear) 
                            remain_time = self.calibration_duration - elapsed_calib
                            cv2.putText(display, f"CALIBRATION: {remain_time:.1f}s", (180, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
                            cv2.putText(display, "Please keep a neutral face & open eyes", (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            continue 
                        else:
                            self.is_calibrating = False
                            if self.baseline_eyebrow_ratios and self.baseline_ears:
                                self.calibrated_eyebrow_ratio = sum(self.baseline_eyebrow_ratios) / len(self.baseline_eyebrow_ratios)
                                self.calibrated_ear = sum(self.baseline_ears) / len(self.baseline_ears)

                    dx = lm[454].x - lm[234].x
                    dz = lm[454].z - lm[234].z
                    yaw_angle = math.degrees(math.atan2(dz, dx)) 
                    
                    is_turning_left = yaw_angle < -25.0  
                    is_turning_right = yaw_angle > 25.0  

                    mouth_open_dist = abs(lm[13].y - lm[14].y)
                    is_mouth_open = (mouth_open_dist / face_height) > 0.05 

                    is_eyebrow_raise = eyebrow_ratio > (self.calibrated_eyebrow_ratio * 1.15)
                    
                    is_wink_left = left_ear < (self.calibrated_ear * 0.65) and right_ear > (self.calibrated_ear * 0.75)
                    is_wink_right = right_ear < (self.calibrated_ear * 0.65) and left_ear > (self.calibrated_ear * 0.75)

                    def check_gesture(name, condition, duration_req):
                        if condition:
                            if self.gesture_timers[name] == 0.0:
                                self.gesture_timers[name] = current_time
                            elapsed = current_time - self.gesture_timers[name]
                            if elapsed >= duration_req: return True, elapsed
                            return False, elapsed
                        else:
                            self.gesture_timers[name] = 0.0
                            return False, 0.0

                    # ---------------------------------------------------------
                    # 3. 상태 머신 로직 (State Machine)
                    # ---------------------------------------------------------
                    
                    # IDLE 상태일 때만 얼굴 제스처를 감지하여 상태를 Lock 합니다.
                    if self.current_state == "IDLE":
                        t_left_trig, t_left_el = check_gesture("TURN_LEFT", is_turning_left, 3.0)
                        t_right_trig, t_right_el = check_gesture("TURN_RIGHT", is_turning_right, 3.0)
                        mouth_trig, mouth_el = check_gesture("MOUTH_OPEN", is_mouth_open, 3.0)
                        brow_trig, brow_el = check_gesture("EYEBROW_RAISE", is_eyebrow_raise, 2.0)
                        wink_l_trig, wink_l_el = check_gesture("WINK_LEFT", is_wink_left, 1.0)
                        wink_r_trig, wink_r_el = check_gesture("WINK_RIGHT", is_wink_right, 1.0)

                        triggered_gesture = None
                        if t_left_trig: triggered_gesture = "TURN_LEFT"
                        elif t_right_trig: triggered_gesture = "TURN_RIGHT"
                        elif mouth_trig: triggered_gesture = "MOUTH_OPEN"
                        elif brow_trig: triggered_gesture = "EYEBROW_RAISE"
                        elif wink_l_trig: triggered_gesture = "WINK_LEFT"
                        elif wink_r_trig: triggered_gesture = "WINK_RIGHT"

                        if triggered_gesture:
                            self.current_state = triggered_gesture # 상태 잠금!
                            self.active_gesture_msg = triggered_gesture
                            self.msg_clear_time = current_time + 2.0 
                            
                            cmd_msg = String()
                            cmd_msg.data = triggered_gesture
                            self.gesture_pub.publish(cmd_msg)
                            self.gesture_timers[triggered_gesture] = 0.0 
                    else:
                        # 잠긴 상태에서는 모든 얼굴 제스처 타이머를 0으로 유지
                        for k in self.gesture_timers: self.gesture_timers[k] = 0.0
                        t_left_el = t_right_el = mouth_el = brow_el = wink_l_el = wink_r_el = 0.0

                    # IDLE이 아닐 때만 주먹 쥐기를 통해 IDLE로 복귀를 허용
                    fist_elapsed = 0.0
                    if self.current_state != "IDLE":
                        if is_right_fist_active:
                            if self.right_fist_timer == 0.0:
                                self.right_fist_timer = current_time
                            fist_elapsed = current_time - self.right_fist_timer
                            if fist_elapsed >= 3.0:
                                self.current_state = "IDLE" # 상태 해제!
                                self.active_gesture_msg = "IDLE_RESET"
                                self.msg_clear_time = current_time + 2.0
                                
                                # 로봇 컨트롤러 쪽에 IDLE 명령 퍼블리시
                                cmd_msg = String()
                                cmd_msg.data = "IDLE"
                                self.gesture_pub.publish(cmd_msg)
                                self.right_fist_timer = 0.0
                        else:
                            self.right_fist_timer = 0.0

                    # ---------------------------------------------------------
                    # 4. 화면 UI 출력 로직
                    # ---------------------------------------------------------
                    text_color = (0, 165, 255) 
                    
                    if current_time < self.msg_clear_time:
                        display_text = f"Action: {self.active_gesture_msg}!"
                        text_color = (0, 0, 255) 
                    elif self.current_state != "IDLE":
                        # 잠긴 상태의 UI 처리
                        if is_right_fist_active:
                            display_text = f"Resetting to IDLE... {fist_elapsed:.1f}s / 3.0s"
                            text_color = (0, 255, 255) # 노란색 텍스트
                        else:
                            display_text = f"LOCKED: {self.current_state} (Right Fist 3s to Reset)"
                            text_color = (0, 0, 255)   # 빨간색 텍스트
                    else:
                        # 대기 상태(IDLE)의 UI 처리
                        if t_left_el > 0: display_text = f"Turning Left... {t_left_el:.1f}s / 3.0s"
                        elif t_right_el > 0: display_text = f"Turning Right... {t_right_el:.1f}s / 3.0s"
                        elif mouth_el > 0: display_text = f"Opening Mouth... {mouth_el:.1f}s / 3.0s"
                        elif brow_el > 0: display_text = f"Raising Brows... {brow_el:.1f}s / 2.0s"
                        elif wink_l_el > 0: display_text = f"Winking Left... {wink_l_el:.1f}s / 1.0s"
                        elif wink_r_el > 0: display_text = f"Winking Right... {wink_r_el:.1f}s / 1.0s"
                        else: display_text = "State: IDLE (Ready for Gesture)"
                        text_color = (0, 255, 0) # 초록색 대기 텍스트

                    cv2.putText(display, display_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

                    if not self.is_calibrating:
                        cv2.putText(display, f"Yaw: {yaw_angle:.1f}", (520, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            elapsed = time.time() - start_time
            fps = 1.0 / elapsed if elapsed > 0 else 30.0
            cv2.putText(display, f"FPS: {fps:.1f} | Hand & Face", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            cv2.imshow('Ultra Light Tracker', display)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            time.sleep(max(0, (1.0 / 30.0) - (time.time() - start_time)))

    def destroy_node(self):
        self.is_running = False
        self.vision_thread.join()
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = UltraLightMultiTracker()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__': main()
