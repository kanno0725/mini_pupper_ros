import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from .Config import Configuration


class AvoidanceNode(Node):

    def __init__(self, config):
        super().__init__('avoidance_node')
        self.config = config

        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.last_scan = None
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(
            self.config.timer_period, self.control_loop)

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def control_loop(self):
        # ガード
        if self.last_scan is None:
            return

        # 無効値を置換
        ranges = np.array(self.last_scan.ranges)
        range_max = self.last_scan.range_max
        range_min = self.last_scan.range_min
        ranges = np.nan_to_num(ranges, nan=range_max, posinf=range_max)
        ranges = np.where(ranges < range_min, range_max, ranges)

        # セクタごとの最小距離を求める
        front_rad = self.config.front_rad
        sector = self.config.sector_width_rad
        front_min = self.calc_sector_min(ranges, front_rad, sector)
        left_min = self.calc_sector_min(ranges, front_rad + sector, sector)
        right_min = self.calc_sector_min(ranges, front_rad - sector, sector)

        # 進む方向を判定してTwistを作成
        twist = Twist()
        if front_min > self.config.obstacle_threshold:
            self.get_logger().info(f'go',throttle_duration_sec=1.0)
            twist.linear.x = float(self.config.forward_speed)   # 前進
            twist.angular.z = float(0.0)
        else:
            self.get_logger().info(f'turn',throttle_duration_sec=1.0)
            twist.linear.x = float(0.0)                     # その場で旋回
            if left_min > right_min:
                twist.angular.z = float(self.config.turn_speed)   # 左旋回
            else:
                twist.angular.z = float(-1 * self.config.turn_speed)   # 右旋回

        # publish
        self.publisher_.publish(twist)

    def calc_sector_min(self, ranges, center_rad, sector_width_rad):
        center_idx = np.round((center_rad / (2*np.pi)) * len(ranges)).astype(int)
        sector_half_idx = np.round((sector_width_rad / (2*np.pi * 2)) * len(ranges)).astype(int)
        idx = np.arange(center_idx - sector_half_idx, center_idx + sector_half_idx + 1) % len(ranges)
        return ranges[idx].min()


def main(args=None):
    rclpy.init(args=args)
    config = Configuration()
    node = AvoidanceNode(config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user, shutting down...")
    finally:
        node.destroy_node()
