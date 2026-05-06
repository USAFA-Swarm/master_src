import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
import csv
import math
import time
from datetime import datetime
from pathlib import Path


class AprilTagLogger(Node):
    def __init__(self):
        super().__init__('apriltag_logger')
        self.declare_parameter('tag_family', '36h11')
        self.declare_parameter('log_dir', '~/apriltag_logs')
        self.declare_parameter('warn_timeout', 2.0)

        self._family = self.get_parameter('tag_family').value
        log_dir = Path(self.get_parameter('log_dir').value).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self._warn_timeout = self.get_parameter('warn_timeout').value

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = log_dir / f'apriltag_{ts}.csv'
        self._start_time = None
        self._csv_file = None
        self._writer = None
        self._row_count = 0
        self._last_detection_time = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections',
            self._detections_cb, 10,
        )
        self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info(
            f'[apriltag_logger] Will write to {self._csv_path} on first detection'
        )

    def _open_csv(self):
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(['timestamp_sec', 'tag_id', 'x_m', 'y_m', 'z_m', 'lateral_m'])
        self._csv_file.flush()
        self.get_logger().info(f'[apriltag_logger] Recording to {self._csv_path}')

    def _detections_cb(self, msg):
        if not msg.detections:
            return

        now = time.monotonic()
        self._last_detection_time = now

        if self._start_time is None:
            self._start_time = now
            self._open_csv()

        elapsed = now - self._start_time

        for d in msg.detections:
            tag_frame = f'tag{self._family}:{d.id}'
            try:
                tf = self._tf_buffer.lookup_transform(
                    'camera', tag_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                z = tf.transform.translation.z
                lateral = math.sqrt(x * x + y * y)

                self._writer.writerow([
                    f'{elapsed:.3f}', d.id,
                    f'{x:.4f}', f'{y:.4f}', f'{z:.4f}', f'{lateral:.4f}',
                ])
                self._row_count += 1
                self.get_logger().info(
                    f'[apriltag_logger] Tag {d.id} | '
                    f'z={z:.3f} m  lateral={lateral:.3f} m  '
                    f'x={x:+.3f} m  y={y:+.3f} m'
                )
            except Exception:
                pass  # TF not yet available for this tag

        if self._csv_file:
            self._csv_file.flush()

    def _watchdog_cb(self):
        if self._last_detection_time is None:
            return
        elapsed = time.monotonic() - self._last_detection_time
        if elapsed > self._warn_timeout:
            self.get_logger().warn(
                f'[apriltag_logger] No detections for {elapsed:.1f}s'
            )

    def destroy_node(self):
        if self._csv_file:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info(
                f'[apriltag_logger] Closed — {self._row_count} rows written to {self._csv_path}'
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
