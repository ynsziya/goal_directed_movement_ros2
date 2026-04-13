import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class RobotControllerV4(Node):
    def __init__(self):
        super().__init__('controller_v4')

        # ---------------- Parameters ----------------
        self.declare_parameter('front_stop_threshold', 0.12)
        self.declare_parameter('front_avoid_threshold', 0.55)
        self.declare_parameter('front_clear_threshold', 0.80)
        self.declare_parameter('side_diff_threshold', 0.10)
        self.declare_parameter('goal_tolerance', 0.20)

        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('avoid_linear_speed', 0.03)
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('avoid_turn_speed', 0.8)

        self.declare_parameter('backup_linear_speed', -0.05)
        self.declare_parameter('backup_duration', 1.0)
        self.declare_parameter('avoid_min_duration', 1.0)

        self.declare_parameter('k_goal', 1.2)

        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)

        self.declare_parameter('debug_log_enabled', True)

        self.front_stop_threshold = self.get_parameter('front_stop_threshold').value
        self.front_avoid_threshold = self.get_parameter('front_avoid_threshold').value
        self.front_clear_threshold = self.get_parameter('front_clear_threshold').value
        self.side_diff_threshold = self.get_parameter('side_diff_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.avoid_linear_speed = self.get_parameter('avoid_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.avoid_turn_speed = self.get_parameter('avoid_turn_speed').value

        self.backup_linear_speed = self.get_parameter('backup_linear_speed').value
        self.backup_duration = self.get_parameter('backup_duration').value
        self.avoid_min_duration = self.get_parameter('avoid_min_duration').value

        self.k_goal = self.get_parameter('k_goal').value

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.debug_log_enabled = self.get_parameter('debug_log_enabled').value

        # ---------------- Subscribers ----------------
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # ---------------- Publishers ----------------
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10
        )

        self.state_publisher = self.create_publisher(
            String,
            'robot_state',
            10
        )

        # ---------------- Internal state ----------------
        self.state = 'IDLE'
        self.state_start_time = self.get_clock().now()

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.front = None
        self.front_min = None
        self.left = None
        self.right = None

        self.last_avoid_state = 'AVOID_LEFT'
        self.last_debug_time = self.get_clock().now()

        self.get_logger().info('Robot controller V4 started')

    # -------------------------------------------------
    # Utility
    # -------------------------------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def time_in_state(self):
        now = self.get_clock().now()
        return (now - self.state_start_time).nanoseconds / 1e9

    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State change: {self.state} -> {new_state}')
            self.state = new_state
            self.state_start_time = self.get_clock().now()

    def get_valid_ranges(self, values, range_min, range_max):
        valid = []
        for v in values:
            if math.isfinite(v) and range_min < v < range_max:
                valid.append(v)
        return valid

    def get_sector_median(self, values, range_min, range_max):
        valid = self.get_valid_ranges(values, range_min, range_max)

        if not valid:
            return range_max

        valid.sort()
        mid = len(valid) // 2

        if len(valid) % 2 == 0:
            return (valid[mid - 1] + valid[mid]) / 2.0

        return valid[mid]

    def get_sector_min(self, values, range_min, range_max):
        valid = self.get_valid_ranges(values, range_min, range_max)
        if not valid:
            return range_max
        return min(valid)

    def compute_regions(self, msg):
        ranges = list(msg.ranges)
        n = len(ranges)

        if n == 0:
            return None

        front_vals = ranges[:10] + ranges[-10:]

        left_center = n // 4
        left_vals = ranges[left_center - 10:left_center + 10]

        right_center = 3 * n // 4
        right_vals = ranges[right_center - 10:right_center + 10]

        front_median = self.get_sector_median(front_vals, msg.range_min, msg.range_max)
        front_min = self.get_sector_min(front_vals, msg.range_min, msg.range_max)
        left_median = self.get_sector_median(left_vals, msg.range_min, msg.range_max)
        right_median = self.get_sector_median(right_vals, msg.range_min, msg.range_max)

        return front_median, front_min, left_median, right_median

    def has_pose(self):
        return (
            self.current_x is not None and
            self.current_y is not None and
            self.current_yaw is not None
        )

    def get_distance_to_goal(self):
        if not self.has_pose():
            return None

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        return math.sqrt(dx * dx + dy * dy)

    def get_goal_yaw(self):
        if not self.has_pose():
            return None

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        return math.atan2(dy, dx)

    def get_yaw_error(self):
        goal_yaw = self.get_goal_yaw()

        if goal_yaw is None or self.current_yaw is None:
            return None

        return self.normalize_angle(goal_yaw - self.current_yaw)

    def choose_avoid_side(self):
        diff = self.left - self.right

        if diff > self.side_diff_threshold:
            return 'AVOID_LEFT'

        if diff < -self.side_diff_threshold:
            return 'AVOID_RIGHT'

        yaw_error = self.get_yaw_error()
        if yaw_error is not None:
            if yaw_error > 0.0:
                return 'AVOID_LEFT'
            return 'AVOID_RIGHT'

        if self.left >= self.right:
            return 'AVOID_LEFT'
        return 'AVOID_RIGHT'

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def scan_callback(self, msg):
        regions = self.compute_regions(msg)

        if regions is None:
            self.get_logger().warn('Empty scan data received')
            return

        self.front, self.front_min, self.left, self.right = regions

        distance_to_goal = self.get_distance_to_goal()
        yaw_error = self.get_yaw_error()

        new_state = self.decide_state(distance_to_goal)
        self.set_state(new_state)

        twist_msg = self.build_twist(distance_to_goal, yaw_error)

        self.cmd_vel_publisher.publish(twist_msg)
        self.publish_state()
        self.maybe_log_debug(distance_to_goal, yaw_error)

    # -------------------------------------------------
    # Decision
    # -------------------------------------------------
    def decide_state(self, distance_to_goal):
        if distance_to_goal is not None and distance_to_goal <= self.goal_tolerance:
            return 'GOAL_REACHED'

        # BACK_UP içindeysek süre dolana kadar devam et
        if self.state == 'BACK_UP':
            if self.time_in_state() < self.backup_duration:
                return 'BACK_UP'
            return self.last_avoid_state

        # Çok yakın engel varsa recovery başlat
        if self.front_min is not None and self.front_min < self.front_stop_threshold:
            if self.state in ['AVOID_LEFT', 'AVOID_RIGHT']:
                self.last_avoid_state = self.state
            return 'BACK_UP'

        # Zaten avoid modundaysak çıkış koşullarına bak
        if self.state in ['AVOID_LEFT', 'AVOID_RIGHT']:
            if self.time_in_state() < self.avoid_min_duration:
                return self.state

            if self.front is not None and self.front < self.front_clear_threshold:
                return self.state

            return 'GO_TO_GOAL'

        # Yeni engel varsa taraf seç
        if self.front is not None and self.front < self.front_avoid_threshold:
            chosen = self.choose_avoid_side()
            self.last_avoid_state = chosen
            return chosen

        return 'GO_TO_GOAL'

    # -------------------------------------------------
    # Control
    # -------------------------------------------------
    def build_twist(self, distance_to_goal, yaw_error):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        linear_x = 0.0
        angular_z = 0.0

        if self.state == 'GOAL_REACHED':
            linear_x = 0.0
            angular_z = 0.0

        elif self.state == 'BACK_UP':
            linear_x = self.backup_linear_speed
            angular_z = 0.0

        elif self.state == 'GO_TO_GOAL':
            if yaw_error is not None:
                angular_z = self.k_goal * yaw_error
            else:
                angular_z = 0.0

            angular_z = self.clamp(
                angular_z,
                -self.max_angular_speed,
                self.max_angular_speed
            )

            if distance_to_goal is None:
                linear_x = self.min_linear_speed
            else:
                linear_x = self.max_linear_speed

                if distance_to_goal < 1.0:
                    linear_x *= distance_to_goal

                if yaw_error is not None:
                    heading_scale = max(0.0, 1.0 - abs(yaw_error) / math.pi)
                    linear_x *= heading_scale

                linear_x = self.clamp(
                    linear_x,
                    self.min_linear_speed,
                    self.max_linear_speed
                )

        elif self.state == 'AVOID_LEFT':
            linear_x = self.avoid_linear_speed
            angular_z = self.avoid_turn_speed

        elif self.state == 'AVOID_RIGHT':
            linear_x = self.avoid_linear_speed
            angular_z = -self.avoid_turn_speed

        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.angular.z = angular_z

        return twist_msg

    # -------------------------------------------------
    # Publishing / Logging
    # -------------------------------------------------
    def publish_state(self):
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher.publish(state_msg)

    def maybe_log_debug(self, distance_to_goal, yaw_error):
        if not self.debug_log_enabled:
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_debug_time).nanoseconds / 1e9

        if elapsed >= 1.0:
            dist_text = 'None' if distance_to_goal is None else f'{distance_to_goal:.2f}'
            yaw_text = 'None' if yaw_error is None else f'{yaw_error:.2f}'
            front_text = 'None' if self.front is None else f'{self.front:.2f}'
            front_min_text = 'None' if self.front_min is None else f'{self.front_min:.2f}'
            left_text = 'None' if self.left is None else f'{self.left:.2f}'
            right_text = 'None' if self.right is None else f'{self.right:.2f}'

            self.get_logger().info(
                f'State: {self.state} | '
                f'FrontMed: {front_text} | FrontMin: {front_min_text} | '
                f'Left: {left_text} | Right: {right_text} | '
                f'Dist: {dist_text} | Yaw error: {yaw_text}'
            )
            self.last_debug_time = now


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerV4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()