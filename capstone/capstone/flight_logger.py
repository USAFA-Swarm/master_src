import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
import csv
import time
from datetime import datetime
from pathlib import Path


class FlightLogger(Node):
    def __init__(self):
        super().__init__('flight_logger')
        self.declare_parameter('drone_name', 'drone1')
        self.declare_parameter('log_dir', '~/flight_logs')

        drone_name = self.get_parameter('drone_name').value
        log_dir = Path(self.get_parameter('log_dir').value).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = log_dir / f'flight_{ts}.csv'
        self._start_time = None
        self._row_count = 0
        self._last_print = 0.0

        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(['timestamp_sec', 'x_m', 'y_m', 'z_m'])
        self._csv_file.flush()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        topic = f'/{drone_name}/local_position/pose'
        self.create_subscription(PoseStamped, topic, self._pose_cb, qos)

        self.get_logger().info(f'[flight_logger] Recording to {self._csv_path}')
        self.get_logger().info(f'[flight_logger] Subscribing to {topic}')

    def _pose_cb(self, msg):
        now = time.monotonic()
        if self._start_time is None:
            self._start_time = now

        elapsed = now - self._start_time
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self._writer.writerow([f'{elapsed:.3f}', f'{x:.4f}', f'{y:.4f}', f'{z:.4f}'])
        self._row_count += 1

        if self._row_count % 20 == 0:
            self._csv_file.flush()

        if now - self._last_print >= 0.5:
            self._last_print = now
            self.get_logger().info(
                f'[flight_logger] t={elapsed:.1f}s  '
                f'x={x:+.2f}  y={y:+.2f}  z={z:+.2f}'
            )

    def destroy_node(self):
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info(
            f'[flight_logger] Closed — {self._row_count} rows written to {self._csv_path}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlightLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
