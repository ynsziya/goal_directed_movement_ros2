import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # ==========================================
        # Default Parameters
        # ==========================================
        self.declare_parameter('front_stop_threshold', 0.20) # 20 cm
        self.declare_parameter('front_clear_threshold', 0.60)
        self.declare_parameter('side_diff_threshold', 0.15)

        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_linear_speed', 0.25)
        self.declare_parameter('turn_angular_speed', 0.60)
        self.declare_parameter('align_angular_speed', 0.40)

        self.declare_parameter('yaw_tolerance', 0.15)
        self.declare_parameter('goal_tolerance', 0.20)

        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 0.0)

        self.declare_parameter('debug_log_enabled', True)

        # ==========================================
        # Get parameters
        # ==========================================
        self.front_stop_threshold = self.get_parameter('front_stop_threshold').value
        self.front_clear_threshold = self.get_parameter('front_clear_threshold').value
        self.side_diff_threshold = self.get_parameter('side_diff_threshold').value

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_linear_speed = self.get_parameter('turn_linear_speed').value
        self.turn_angular_speed = self.get_parameter('turn_angular_speed').value
        self.align_angular_speed = self.get_parameter('align_angular_speed').value

        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.debug_log_enabled = self.get_parameter('debug_log_enabled').value

        # ==========================================
        # Subscribers
        # ==========================================
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

        # ==========================================
        # Publishers
        # ==========================================
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

        # ==========================================
        # Internal State
        # ==========================================
        self.state = 'IDLE'

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.last_debug_time = self.get_clock().now()

        self.get_logger().info('Goal navigation controller started')

    
    # ==========================================
    # Utility Functions
    # ==========================================
    def clean_ranges(self, values):
        return [v for v in values if math.isfinite(v)]
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z  + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def compute_regions(self, msg):
        ranges = list(msg.ranges)
        n = len(ranges)

        if n == 0:
            return None
        
        front_vals = ranges[:10] + ranges[-10:]

        left_center = n // 4
        left_vals = ranges[left_center - 10 : left_center + 10]

        right_center = 3 * n // 4
        right_vals = ranges[right_center - 10 : right_center +10]

        front = min(self.clean_ranges(front_vals), default=msg.range_max)
        left = min(self.clean_ranges(left_vals), default=msg.range_max)
        right = min(self.clean_ranges(right_vals), default=msg.range_max)

        return front, left, right
    
    def has_pose(self):
        return(
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

    def decide_avoidance_direction(self, left, right):
        diff = left - right

        if diff > self.side_diff_threshold:
            return 'TURN_LEFT'

        if diff < -self.side_diff_threshold:
            return 'TURN_RIGHT'

        if self.state in ['TURN_LEFT', 'TURN_RIGHT']:
            return self.state

        return 'TURN_LEFT'

    # ==========================================
    # Callbacks
    # ==========================================
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
        
        front, left, right = regions

        distance_to_goal = self.get_distance_to_goal()
        yaw_error = self.get_yaw_error()

        new_state = self.decide_state(front, left, right, distance_to_goal, yaw_error)

        if new_state != self.state:
            self.get_logger().info(f'State change: {self.state} -> {new_state}')
            self.state = new_state

        twist_msg = self.build_twist(yaw_error)

        self.cmd_vel_publisher.publish(twist_msg)
        self.publish_state()
        self.maybe_log_debug(front, left, right, distance_to_goal, yaw_error)

    # ==========================================
    # Decision logic
    # ==========================================
    def decide_state(self, front, left, right, distance_to_goal, yaw_error):
        if distance_to_goal is not None and distance_to_goal <= self.goal_tolerance:
            return 'GOAL_REACHED'
        
        if front < self.front_stop_threshold:
            return 'EMERGENCY_STOP'
        
        if front <= self.front_clear_threshold:
            return self.decide_avoidance_direction(left, right)
        
        if yaw_error is None:
            return 'FORWARD'
        
        if abs(yaw_error) > self.yaw_tolerance:
            return 'ALIGN_TO_GOAL'
        
        return 'FORWARD'
    
    # ==========================================
    # Control logic
    # ==========================================
    def build_twist(self, yaw_error):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        if self.state == 'FORWARD':
            twist_msg.twist.linear.x = self.forward_speed
            twist_msg.twist.angular.z = 0.0

        elif self.state == 'TURN_LEFT':
            twist_msg.twist.linear.x = self.turn_linear_speed
            twist_msg.twist.angular.z = self.turn_angular_speed

        elif self.state == 'TURN_RIGHT':
            twist_msg.twist.linear.x = self.turn_linear_speed
            twist_msg.twist.angular.z = -self.turn_angular_speed

        elif self.state == 'ALIGN_TO_GOAL':
            twist_msg.twist.linear.x = 0.0

            if yaw_error is not None:
                if yaw_error > 0:
                    twist_msg.twist.angular.z = self.align_angular_speed
                else:
                    twist_msg.twist.angular.z = -self.align_angular_speed

            else:
                twist_msg.twist.angular.z = 0.0

        elif self.state in ['EMERGENCY_STOP', 'IDLE', 'GOAL_REACHED']:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0

        return twist_msg
    
    # ==========================================
    # Publishing and logging
    # ==========================================
    def publish_state(self):
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher.publish(state_msg)

    def maybe_log_debug(self, front, left, right, distance_to_goal, yaw_error):
        if not self.debug_log_enabled:
            return None
        
        now = self.get_clock().now()
        elapsed = (now - self.last_debug_time).nanoseconds / 1e9

        if elapsed >= 1.0:
            distance_text = 'None' if distance_to_goal is None else f'{distance_to_goal:.2f}'
            yaw_text = 'None' if yaw_error is None else f'{yaw_error:.2f}'

            self.get_logger().info(
                f'Front: {front:.2f} | Left: {left:.2f} | Right: {right:.2f} | '
                f'Dist goal: {distance_text} | Yaw error: {yaw_text} | State: {self.state}'
            )
            self.last_debug_time = now

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


        