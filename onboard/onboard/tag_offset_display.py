"""
tag_offset_display.py — Diagnostic display of drone-to-tag offset with landing simulation.

Reads tag position from the TF transform camera → tag36h11:{id}, which
apriltag_ros broadcasts per detection (computed from tag size + pixel corners).

Camera frame (optical convention):
  x = lateral right
  y = lateral down
  z = depth / range to tag  (≈ altitude above tag for a downward-facing camera)

Simulation mode mirrors precision_landing logic using range (z) as altitude proxy:
  - tracks confirm count toward takeover threshold
  - shows which tag is active (outer vs center) based on range vs tag_switch_alt
  - shows lateral correction that would be commanded (camera frame, metres)
  - shows range vs descent thresholds

No drone pose, TF map→base_link, or fake publishers required.
"""

import math
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray


class TagOffsetDisplay(Node):

    def __init__(self):
        super().__init__('tag_offset_display')

        self.declare_parameter('tag_family',      '36h11')
        self.declare_parameter('display_rate',    2.0)
        self.declare_parameter('warn_timeout',    2.0)
        # Mirror precision_landing params for simulation
        self.declare_parameter('landing_tag_id',  0)
        self.declare_parameter('high_alt_tag_id', 107)
        self.declare_parameter('tag_switch_alt',  0.8)
        self.declare_parameter('confirm_frames',  5)
        self.declare_parameter('lateral_gain',    0.6)
        self.declare_parameter('descent_step',    0.05)
        self.declare_parameter('land_final_alt',  0.4)

        self._last_tag_ids     = []
        self._last_msg_sec     = 0.0
        self._ever_received    = False
        self._confirm_count    = 0
        self._confirmed_tag_id = None  # tag ID the sim has locked onto

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections',
            self._detections_cb, 10)

        rate = self.get_parameter('display_rate').value
        self.create_timer(1.0 / rate, self._display_cb)

        self.get_logger().info('tag_offset_display ready — watching /apriltag/detections')

    def _detections_cb(self, msg):
        self._ever_received = True
        self._last_msg_sec  = self._now()
        self._last_tag_ids  = [d.id for d in msg.detections]

    def _display_cb(self):
        if not self._ever_received:
            return

        elapsed    = self._now() - self._last_msg_sec
        warn_to    = self.get_parameter('warn_timeout').value
        family     = self.get_parameter('tag_family').value
        land_id    = self.get_parameter('landing_tag_id').value
        high_id    = self.get_parameter('high_alt_tag_id').value
        switch_alt = self.get_parameter('tag_switch_alt').value
        confirm_n  = self.get_parameter('confirm_frames').value
        gain       = self.get_parameter('lateral_gain').value
        d_step     = self.get_parameter('descent_step').value
        final_alt  = self.get_parameter('land_final_alt').value

        if not self._last_tag_ids or elapsed > warn_to:
            self._confirm_count    = 0
            self._confirmed_tag_id = None
            self.get_logger().warn(
                f'[tag_offset] no detections for {elapsed:.1f} s',
                throttle_duration_sec=1.0)
            return

        # Look up pose for every visible tag
        poses = {}
        for tid in self._last_tag_ids:
            tag_frame = f'tag{family}:{tid}'
            try:
                tf = self._tf_buffer.lookup_transform(
                    'camera', tag_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
                poses[tid] = (
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

        if not poses:
            self.get_logger().warn('[tag_offset] detections present but TF unavailable')
            return

        # Raw offset for every visible tag
        for tid, (x, y, z) in poses.items():
            lateral = math.sqrt(x * x + y * y)
            self.get_logger().info(
                f'[tag_offset] Tag {tid:3d} | '
                f'range={z:.3f} m  lateral={lateral:.3f} m  '
                f'x={x:+.3f} m  y={y:+.3f} m')

        # --- Simulation: mirror precision_landing decision logic ---

        # IDLE phase: accept any visible tag, track whichever is seen consistently
        if self._confirmed_tag_id is None:
            if self._last_tag_ids:
                first_id = self._last_tag_ids[0]
                if not hasattr(self, '_sim_last_tag') or self._sim_last_tag != first_id:
                    self._confirm_count  = 0
                    self._sim_last_tag   = first_id
                self._confirm_count += 1
                if self._confirm_count >= confirm_n:
                    self._confirmed_tag_id = first_id
                    self.get_logger().warn(
                        f'[sim] Tag {first_id} confirmed — WOULD TAKE OVER')
                else:
                    self.get_logger().info(
                        f'[sim] Tag {first_id} — searching ({self._confirm_count}/{confirm_n})')
            return

        # CENTERING phase: use confirmed tag, switch to landing_tag below switch_alt
        ref_range = poses.get(self._confirmed_tag_id, (None, None, None))[2]
        if ref_range is not None and ref_range < switch_alt:
            sim_active = land_id
        else:
            sim_active = self._confirmed_tag_id

        if sim_active not in poses:
            self.get_logger().warn(
                f'[sim] Tag {sim_active} not in TF — would hold position')
            return

        x, y, z = poses[sim_active]
        lateral = math.sqrt(x * x + y * y)
        corr_x  = -x * gain
        corr_y  = -y * gain

        if z <= final_alt:
            self.get_logger().info(
                f'[sim] range={z:.3f} m <= land_final_alt={final_alt} m '
                f'— WOULD HANDOFF TO LAND')
        else:
            next_range = max(z - d_step, final_alt)
            self.get_logger().info(
                f'[sim] centering on tag {sim_active} | '
                f'correction: cam_x={corr_x:+.3f} m  cam_y={corr_y:+.3f} m  '
                f'descend {d_step:.3f} m → range={next_range:.3f} m '
                f'(lateral err={lateral:.3f} m)')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = TagOffsetDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
