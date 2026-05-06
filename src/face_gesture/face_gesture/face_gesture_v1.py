import sys
import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String  # 제스처 명령 전송용
import threading
import time

class UltraLightMultiTracker(Node):
    def __init__(self):
        super().__init__('ultra_light_tracker')
        self.left_pub = self.create_publisher(Point, '/left_hand', 10)
        self.right_pub = self.create_publisher(Point, '/right_hand', 10)
        self.face_pub = self.create_publisher(Point, '/face_pose', 10)
        
        # [추가] 얼굴 제스처 명령 퍼블리셔
        self.gesture_pub = self.create_publisher(String, '/face_gesture_cmd', 10)

        self.mp_hands = mp.solutions.hands
        self.mp_face_mesh = mp.solutions.face_mesh
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.hands = self.mp_hands.Hands(max_num_hands=2, model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        # refine_landmarks=True 로 설정되어 있어 478개 랜드마크(눈동자 포함) 추출 가능
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
        
        # 제스처 중복 인식을 막기 위한 쿨다운 변수
        self.last_gesture_time = time.time()
        self.gesture_cooldown = 1.0  # 1초에 한 번만 제스처 발행

        self.vision_thread = threading.Thread(target=self.camera_loop)
        self.vision_thread.start()

    # 눈 깜빡임(EAR) 계산 함수
    def get_eye_aspect_ratio(self, landmarks, top, bottom, left, right):
        v_dist = np.linalg.norm(np.array([landmarks[top].x, landmarks[top].y]) - 
                                np.array([landmarks[bottom].x, landmarks[bottom].y]))
        h_dist = np.linalg.norm(np.array([landmarks[left].x, landmarks[left].y]) - 
                                np.array([landmarks[right].x, landmarks[right].y]))
        return v_dist / h_dist if h_dist > 0 else 0

    def camera_loop(self):
        while self.is_running and rclpy.ok():
            start_time = time.time()
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame, depth_frame = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color_frame or not depth_frame: continue

            image = np.asanyarray(color_frame.get_data())
            display = cv2.flip(image, 1) # 거울 모드 (사용자의 왼쪽이 화면의 왼쪽)
            rgb_display = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            
            hand_results = self.hands.process(rgb_display)
            face_results = self.face_mesh.process(rgb_display)
            
            current_time = time.time()
            gesture_text = "None"

            if face_results.multi_face_landmarks:
                for face_landmarks in face_results.multi_face_landmarks:
                    
                    # --- [추가] MediaPipe가 제공하는 모든 얼굴 랜드마크 시각화 ---
                    # 1. 얼굴 전체 그물망 (Tessellation)
                    self.mp_drawing.draw_landmarks(
                        image=display,
                        landmark_list=face_landmarks,
                        connections=self.mp_face_mesh.FACEMESH_TESSELATION,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_tesselation_style()
                    )
                    # 2. 눈, 눈썹, 입술, 얼굴 윤곽선 (Contours)
                    self.mp_drawing.draw_landmarks(
                        image=display,
                        landmark_list=face_landmarks,
                        connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style()
                    )
                    # 3. 눈동자 (Irises)
                    self.mp_drawing.draw_landmarks(
                        image=display,
                        landmark_list=face_landmarks,
                        connections=self.mp_face_mesh.FACEMESH_IRISES,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_iris_connections_style()
                    )
                    # -----------------------------------------------------------

                    # 1번 랜드마크(코 끝) 3D 좌표 추출 및 ROS 퍼블리시
                    nx, ny = int(face_landmarks.landmark[1].x * 640), int(face_landmarks.landmark[1].y * 480)
                    if 20 < nx < 620 and 20 < ny < 460:
                        depth_nx = min(639, max(0, 639 - nx))
                        face_dist = depth_frame.get_distance(depth_nx, ny)
                        if 0.2 < face_dist < 2.5:
                            p3d = rs.rs2_deproject_pixel_to_point(self.intr, [depth_nx, ny], face_dist)
                            self.face_pub.publish(Point(x=float(p3d[0]), y=float(p3d[1]), z=float(p3d[2])))
                            # 코 끝에 약간 큰 노란 점 표시 (전체 메쉬 중 코 끝 위치 강조)
                            cv2.circle(display, (nx, ny), 6, (0, 255, 255), -1)

                    # --- 제스처 인식 로직 ---
                    lm = face_landmarks.landmark
                    
                    # 1. 눈 깜빡임 인식 (EAR 비율 계산)
                    left_ear = self.get_eye_aspect_ratio(lm, top=159, bottom=145, left=33, right=133)
                    right_ear = self.get_eye_aspect_ratio(lm, top=386, bottom=374, left=362, right=263)
                    
                    EAR_THRESHOLD = 0.20  # 이 값보다 작으면 눈을 감은 것으로 간주
                    
                    # 2. 고개 돌림 인식
                    dist_to_right_edge = abs(lm[1].x - lm[234].x) 
                    dist_to_left_edge = abs(lm[1].x - lm[454].x)
                    
                    YAW_THRESHOLD = 2.0  # 비율 차이가 2배 이상 나면 고개를 돌린 것으로 간주
                    
                    if current_time - self.last_gesture_time > self.gesture_cooldown:
                        cmd_msg = String()
                        
                        if left_ear < EAR_THRESHOLD and right_ear > EAR_THRESHOLD:
                            gesture_text = "LEFT_BLINK (Force 10N)"
                            cmd_msg.data = "LEFT_BLINK"
                            self.gesture_pub.publish(cmd_msg)
                            self.last_gesture_time = current_time
                            
                        elif right_ear < EAR_THRESHOLD and left_ear > EAR_THRESHOLD:
                            gesture_text = "RIGHT_BLINK"
                            cmd_msg.data = "RIGHT_BLINK"
                            self.gesture_pub.publish(cmd_msg)
                            self.last_gesture_time = current_time
                            
                        elif dist_to_right_edge / (dist_to_left_edge + 1e-6) > YAW_THRESHOLD:
                            gesture_text = "TURN_LEFT (Force 10N)"
                            cmd_msg.data = "TURN_LEFT"
                            self.gesture_pub.publish(cmd_msg)
                            self.last_gesture_time = current_time
                            
                        elif dist_to_left_edge / (dist_to_right_edge + 1e-6) > YAW_THRESHOLD:
                            gesture_text = "TURN_RIGHT"
                            cmd_msg.data = "TURN_RIGHT"
                            self.gesture_pub.publish(cmd_msg)
                            self.last_gesture_time = current_time

                    # 화면에 제스처 상태 출력
                    cv2.putText(display, f"Gesture: {gesture_text}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            # --- Hand Tracking 로직 (모든 손 랜드마크 그리기 포함됨) ---
            if hand_results.multi_hand_landmarks:
                for idx, hand_landmarks in enumerate(hand_results.multi_hand_landmarks):
                    label = hand_results.multi_handedness[idx].classification[0].label 
                    
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
                        
                        # 손가락의 모든 점 21개와 뼈대(Connections) 그리기
                        self.mp_drawing.draw_landmarks(
                            display, 
                            hand_landmarks, 
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style()
                        )
                        
                        text_color = (0, 0, 255) if label == "Left" else (0, 255, 0)
                        cv2.putText(display, f"{label}: {final_dist:.2f}m", (cx - 40, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)

            elapsed = time.time() - start_time
            fps = 1.0 / elapsed if elapsed > 0 else 30.0
            cv2.putText(display, f"FPS: {fps:.1f} | L:Red, R:Green, Face:Yellow", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

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
