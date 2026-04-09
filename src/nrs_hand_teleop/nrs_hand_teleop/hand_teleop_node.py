import json
import math
from typing import Dict, List, Optional, Tuple


import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Float64MultiArray
from dualarm_forcecon_interfaces.srv import SetControlMode


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def vec_sub(a: Dict[str, float], b: Dict[str, float]) -> Tuple[float, float, float]:
    return (a["x"] - b["x"], a["y"] - b["y"], a["z"] - b["z"])


def vec_norm(v: Tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vec_dot(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def angle_deg_from_points(a: Dict[str, float], b: Dict[str, float], c: Dict[str, float]) -> float:
    """
    Returns angle ABC in degrees.
    """
    ba = vec_sub(a, b)
    bc = vec_sub(c, b)

    nba = vec_norm(ba)
    nbc = vec_norm(bc)

    if nba < 1e-9 or nbc < 1e-9:
        return 180.0

    cosang = clamp(vec_dot(ba, bc) / (nba * nbc), -1.0, 1.0)
    return math.degrees(math.acos(cosang))


class MediaPipeHandTeleopNode(Node):
    def __init__(self):
        super().__init__('nrs_hand_teleop_node')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('landmarks_topic', '/mediapipe_hand_landmarks')
        self.declare_parameter('target_topic', '/forward_hand_joint_targets')
        self.declare_parameter('publish_hz', 20.0)

        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('lost_hold_sec', 0.7)

        # Thumb limits
        self.declare_parameter('thumb1_min', -0.5236)
        self.declare_parameter('thumb1_max',  0.5236)
        self.declare_parameter('thumb2_min',  0.0)
        self.declare_parameter('thumb2_max',  1.0472)
        self.declare_parameter('thumb3_min',  0.0)
        self.declare_parameter('thumb3_max',  1.0472)

        # Other finger limits
        self.declare_parameter('finger1_min', -0.5236)
        self.declare_parameter('finger1_max',  0.5236)
        self.declare_parameter('finger2_min',  0.0)
        self.declare_parameter('finger2_max',  1.5708)
        self.declare_parameter('finger3_min',  0.0)
        self.declare_parameter('finger3_max',  1.1345)

        # Gains
        self.declare_parameter('thumb1_gain', 1.0)
        self.declare_parameter('thumb_flex_gain', 1.0)
        self.declare_parameter('finger_flex_gain', 1.0)

        # Optional sign flip for thumb spread
        self.declare_parameter('left_thumb1_sign', 1.0)
        self.declare_parameter('right_thumb1_sign', 1.0)

        self.landmarks_topic = self.get_parameter('landmarks_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.lost_hold_sec = float(self.get_parameter('lost_hold_sec').value)

        self.thumb1_min = float(self.get_parameter('thumb1_min').value)
        self.thumb1_max = float(self.get_parameter('thumb1_max').value)
        self.thumb2_min = float(self.get_parameter('thumb2_min').value)
        self.thumb2_max = float(self.get_parameter('thumb2_max').value)
        self.thumb3_min = float(self.get_parameter('thumb3_min').value)
        self.thumb3_max = float(self.get_parameter('thumb3_max').value)

        self.finger1_min = float(self.get_parameter('finger1_min').value)
        self.finger1_max = float(self.get_parameter('finger1_max').value)
        self.finger2_min = float(self.get_parameter('finger2_min').value)
        self.finger2_max = float(self.get_parameter('finger2_max').value)
        self.finger3_min = float(self.get_parameter('finger3_min').value)
        self.finger3_max = float(self.get_parameter('finger3_max').value)

        self.thumb1_gain = float(self.get_parameter('thumb1_gain').value)
        self.thumb_flex_gain = float(self.get_parameter('thumb_flex_gain').value)
        self.finger_flex_gain = float(self.get_parameter('finger_flex_gain').value)

        self.left_thumb1_sign = float(self.get_parameter('left_thumb1_sign').value)
        self.right_thumb1_sign = float(self.get_parameter('right_thumb1_sign').value)

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.landmarks_sub = self.create_subscription(
            String,
            self.landmarks_topic,
            self.landmarks_callback,
            10
        )

        self.target_pub = self.create_publisher(
            Float64MultiArray,
            self.target_topic,
            10
        )

        self.mode_client = self.create_client(SetControlMode, '/change_control_mode')

        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)

        # ----------------------------
        # Internal state
        # ----------------------------
        self.latest_left: Optional[List[Dict[str, float]]] = None
        self.latest_right: Optional[List[Dict[str, float]]] = None

        now = self.get_clock().now()
        self.last_left_stamp = now
        self.last_right_stamp = now
        self.last_mode_request_time = now - Duration(seconds=10.0)

        # left15 + right15
        self.q_target = [0.0] * 30

        self.get_logger().info('nrs_hand_teleop node started.')

    # -------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------
    def landmarks_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse /mediapipe_hand_landmarks JSON: {e}')
            return

        now = self.get_clock().now()

        left = data.get('left', None)
        right = data.get('right', None)

        if isinstance(left, list) and len(left) == 21:
            self.latest_left = left
            self.last_left_stamp = now

        if isinstance(right, list) and len(right) == 21:
            self.latest_right = right
            self.last_right_stamp = now

    def timer_callback(self):
        self.ensure_forward_mode()

        now = self.get_clock().now()

        left_valid = (now - self.last_left_stamp) < Duration(seconds=self.lost_hold_sec)
        right_valid = (now - self.last_right_stamp) < Duration(seconds=self.lost_hold_sec)

        if left_valid and self.latest_left is not None:
            q_left = self.compute_hand15(self.latest_left, handedness='Left')
            self.q_target[0:15] = self.ema(self.q_target[0:15], q_left, self.ema_alpha)

        if right_valid and self.latest_right is not None:
            q_right = self.compute_hand15(self.latest_right, handedness='Right')
            self.q_target[15:30] = self.ema(self.q_target[15:30], q_right, self.ema_alpha)

        msg = Float64MultiArray()
        msg.data = self.q_target
        self.target_pub.publish(msg)

    # -------------------------------------------------
    # Service
    # -------------------------------------------------
    def ensure_forward_mode(self):
        now = self.get_clock().now()

        if (now - self.last_mode_request_time) < Duration(seconds=2.0):
            return

        if not self.mode_client.service_is_ready():
            self.get_logger().warn('/change_control_mode service not ready yet')
            self.last_mode_request_time = now
            return

        req = SetControlMode.Request()
        req.arm_mode = 'idle'
        req.hand_mode = 'forward'

        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_response_callback)
        self.last_mode_request_time = now

    def mode_response_callback(self, future):
        try:
            res = future.result()
            if res is not None:
                self.get_logger().info(f'Control mode response: success={res.success}, message="{res.message}"')
            else:
                self.get_logger().warn('Control mode response is None')
        except Exception as e:
            self.get_logger().warn(f'/change_control_mode call failed: {e}')

    # -------------------------------------------------
    # Mapping helpers
    # -------------------------------------------------
    def ema(self, prev: List[float], cur: List[float], alpha: float) -> List[float]:
        return [(1.0 - alpha) * p + alpha * c for p, c in zip(prev, cur)]

    def normalize_angle_to_flex(self, ang_deg: float, open_deg: float, close_deg: float) -> float:
        """
        Open hand  -> angle near 180
        Closed hand -> smaller angle
        Returns flex in [0,1]
        """
        value = (open_deg - ang_deg) / (open_deg - close_deg + 1e-9)
        return clamp(value, 0.0, 1.0)

    def compute_non_thumb_flex(self, lm: List[Dict[str, float]], ids: Tuple[int, int, int, int]) -> float:
        """
        ids = (mcp, pip, dip, tip)
        """
        mcp, pip, dip, tip = ids

        wrist = lm[0]
        p_mcp = lm[mcp]
        p_pip = lm[pip]
        p_dip = lm[dip]
        p_tip = lm[tip]

        a_mcp = angle_deg_from_points(wrist, p_mcp, p_pip)
        a_pip = angle_deg_from_points(p_mcp, p_pip, p_dip)
        a_dip = angle_deg_from_points(p_pip, p_dip, p_tip)

        f_mcp = self.normalize_angle_to_flex(a_mcp, open_deg=175.0, close_deg=85.0)
        f_pip = self.normalize_angle_to_flex(a_pip, open_deg=175.0, close_deg=70.0)
        f_dip = self.normalize_angle_to_flex(a_dip, open_deg=175.0, close_deg=90.0)

        flex = 0.20 * f_mcp + 0.50 * f_pip + 0.30 * f_dip
        return clamp(flex * self.finger_flex_gain, 0.0, 1.0)

    def compute_thumb_flex(self, lm: List[Dict[str, float]]) -> float:
        """
        Thumb landmarks:
        1: CMC, 2: MCP, 3: IP, 4: TIP
        """
        a1 = angle_deg_from_points(lm[1], lm[2], lm[3])
        a2 = angle_deg_from_points(lm[2], lm[3], lm[4])

        f1 = self.normalize_angle_to_flex(a1, open_deg=170.0, close_deg=95.0)
        f2 = self.normalize_angle_to_flex(a2, open_deg=170.0, close_deg=80.0)

        flex = 0.45 * f1 + 0.55 * f2
        return clamp(flex * self.thumb_flex_gain, 0.0, 1.0)

    def compute_thumb_spread(self, lm: List[Dict[str, float]]) -> float:
        """
        Simple thumb spread/opposition surrogate using 2D angle between:
          wrist -> thumb_mcp
          wrist -> index_mcp
        Returns roughly [-1, 1]
        """
        wrist = lm[0]
        thumb_mcp = lm[2]
        index_mcp = lm[5]

        v1 = (thumb_mcp["x"] - wrist["x"], thumb_mcp["y"] - wrist["y"])
        v2 = (index_mcp["x"] - wrist["x"], index_mcp["y"] - wrist["y"])

        n1 = math.hypot(v1[0], v1[1])
        n2 = math.hypot(v2[0], v2[1])

        if n1 < 1e-9 or n2 < 1e-9:
            return 0.0

        c = clamp((v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2), -1.0, 1.0)
        ang = math.degrees(math.acos(c))

        spread_n = clamp((ang - 15.0) / (65.0 - 15.0), 0.0, 1.0)
        return 2.0 * spread_n - 1.0

    def map_finger(self, flex: float) -> Tuple[float, float, float]:
        """
        For initial stable version:
        - joint1 small or zero
        - joint2 main flexion
        - joint3 secondary flexion
        """
        q1 = 0.0
        q2 = self.finger2_min + flex * (self.finger2_max - self.finger2_min)
        q3 = self.finger3_min + flex * (self.finger3_max - self.finger3_min)

        q1 = clamp(q1, self.finger1_min, self.finger1_max)
        q2 = clamp(q2, self.finger2_min, self.finger2_max)
        q3 = clamp(q3, self.finger3_min, self.finger3_max)

        return q1, q2, q3

    def compute_hand15(self, lm: List[Dict[str, float]], handedness: str) -> List[float]:
        """
        Output order:
        [thumb1, thumb2, thumb3,
         index1, index2, index3,
         middle1, middle2, middle3,
         ring1, ring2, ring3,
         baby1, baby2, baby3]
        """
        thumb_flex = self.compute_thumb_flex(lm)
        thumb_spread = self.compute_thumb_spread(lm)

        sign = self.left_thumb1_sign if handedness == 'Left' else self.right_thumb1_sign

        q_thumb1_center = 0.5 * (self.thumb1_min + self.thumb1_max)
        q_thumb1_half = 0.5 * (self.thumb1_max - self.thumb1_min)
        q_thumb1 = q_thumb1_center + sign * self.thumb1_gain * q_thumb1_half * thumb_spread
        q_thumb1 = clamp(q_thumb1, self.thumb1_min, self.thumb1_max)

        q_thumb2 = self.thumb2_min + thumb_flex * (self.thumb2_max - self.thumb2_min)
        q_thumb3 = self.thumb3_min + thumb_flex * (self.thumb3_max - self.thumb3_min)
        q_thumb2 = clamp(q_thumb2, self.thumb2_min, self.thumb2_max)
        q_thumb3 = clamp(q_thumb3, self.thumb3_min, self.thumb3_max)

        flex_index = self.compute_non_thumb_flex(lm, (5, 6, 7, 8))
        flex_middle = self.compute_non_thumb_flex(lm, (9, 10, 11, 12))
        flex_ring = self.compute_non_thumb_flex(lm, (13, 14, 15, 16))
        flex_baby = self.compute_non_thumb_flex(lm, (17, 18, 19, 20))

        q_index1, q_index2, q_index3 = self.map_finger(flex_index)
        q_middle1, q_middle2, q_middle3 = self.map_finger(flex_middle)
        q_ring1, q_ring2, q_ring3 = self.map_finger(flex_ring)
        q_baby1, q_baby2, q_baby3 = self.map_finger(flex_baby)

        return [
            q_thumb1, q_thumb2, q_thumb3,
            q_index1, q_index2, q_index3,
            q_middle1, q_middle2, q_middle3,
            q_ring1, q_ring2, q_ring3,
            q_baby1, q_baby2, q_baby3,
        ]


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeHandTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()