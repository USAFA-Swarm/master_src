"""
precision_landing.py — Autonomous precision landing node

Runs on Pi. Monitors /apriltag/detections at all times. When the target
landing tag is seen for confirm_frames consecutive frames, this node:
  1. Publishes /<drone>/precision_landing/takeover = True
     (controller.py stops publishing setpoints and yields)
  2. Uses TF2 (map → tag) to compute lateral error in ENU world frame
  3. Sends corrective /<drone>/setpoint_position/local setpoints while descending
  4. Below land_final_alt, requests LAND mode and releases control

If the tag is lost mid-descent the node holds current position until the
tag reappears or tag_loss_timeout expires (then just holds — operator must
intervene via /<drone>/precision_landing/abort or manual RC takeover).

State machine:
  IDLE → CENTERING → HANDOFF → IDLE
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from apriltag_msgs.msg import AprilTagDetectionArray
from mavros_msgs.srv import SetMode

import tf2_ros
import rclpy.time
import rclpy.duration


class _State:
    IDLE = 'IDLE'
    CENTERING = 'CENTERING'
    HANDOFF = 'HANDOFF'


class PrecisionLanding(Node):

    def __init__(self):
        super().__init__('precision_landing')

        # Parameters
        self.declare_parameter('drone_name',       'drone1')
        self.declare_parameter('landing_tag_id',   0)
        self.declare_parameter('high_alt_tag_id',  4)
        self.declare_parameter('tag_switch_alt',   0.8)
        self.declare_parameter('confirm_frames',   5)
        self.declare_parameter('tag_loss_timeout', 3.0)
        self.declare_parameter('descent_step',     0.05)
        self.declare_parameter('land_final_alt',   0.4)
        self.declare_parameter('lateral_gain',     0.6)
        self.declare_parameter('loop_rate',        10.0)

        drone               = self.get_parameter('drone_name').value
        self._drone         = drone
        self._tag_id        = self.get_parameter('landing_tag_id').value
        self._high_tag_id   = self.get_parameter('high_alt_tag_id').value
        self._switch_alt    = self.get_parameter('tag_switch_alt').value
        self._confirm_n     = self.get_parameter('confirm_frames').value
        self._loss_to       = self.get_parameter('tag_loss_timeout').value
        self._d_step        = self.get_parameter('descent_step').value
        self._final_alt     = self.get_parameter('land_final_alt').value
        self._gain          = self.get_parameter('lateral_gain').value
        loop_hz             = self.get_parameter('loop_rate').value

        # Internal state
        self._pl_state        = _State.IDLE
        self._confirm_count   = 0
        self._last_confirm_tag = None  # which tag ID the confirm counter is for
        self._current_pose    = None   # latest PoseStamped from MAVROS
        self._hold_pose       = None   # PoseStamped to hold when tag lost
        self._last_tag_sec    = 0.0    # clock time of last confirmed tag detection
        self._handoff_sent    = False

        # QoS matching MAVROS (BEST_EFFORT)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # TF2
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscriptions
        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections',
            self._detections_cb, 10)
        self.create_subscription(
            PoseStamped, f'/{drone}/local_position/pose',
            self._pose_cb, qos)
        self.create_subscription(
            Bool, f'/{drone}/precision_landing/abort',
            self._abort_cb, 10)

        # Publications
        self._takeover_pub  = self.create_publisher(
            Bool, f'/{drone}/precision_landing/takeover', 10)
        self._setpoint_pub  = self.create_publisher(
            PoseStamped, f'/{drone}/setpoint_position/local', qos)

        # Service client for LAND mode handoff
        self._mode_client = self.create_client(SetMode, f'/{drone}/set_mode')

        # Control loop timer
        self.create_timer(1.0 / loop_hz, self._control_loop)

        self.get_logger().info(
            f'PrecisionLanding ready — drone={drone}, tag_id={self._tag_id}')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _active_tag_id(self) -> int:
        """Return which tag ID to track based on current altitude.

        Above tag_switch_alt: use high_alt_tag_id (large outer ring, ID 4).
        Below tag_switch_alt: use landing_tag_id (small center tag, ID 0).
        If altitude unknown, default to outer tag.
        """
        if self._current_pose is None:
            return self._high_tag_id
        z = self._current_pose.pose.position.z
        return self._tag_id if z < self._switch_alt else self._high_tag_id

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _pose_cb(self, msg):
        self._current_pose = msg

    def _abort_cb(self, msg):
        if msg.data and self._pl_state != _State.IDLE:
            self.get_logger().warn('[PL] Abort received — releasing control')
            self._release_control()

    def _detections_cb(self, msg):
        active = self._active_tag_id()
        found  = any(d.id == active for d in msg.detections)

        if self._pl_state == _State.IDLE:
            # Reset counter if altitude changed and we're now watching a different tag
            if self._last_confirm_tag != active:
                self._confirm_count    = 0
                self._last_confirm_tag = active

            if found:
                self._confirm_count += 1
                self.get_logger().debug(
                    f'[PL] Tag {active} seen {self._confirm_count}/{self._confirm_n}')
                if self._confirm_count >= self._confirm_n:
                    self._take_control()
            else:
                self._confirm_count = 0

        elif self._pl_state == _State.CENTERING:
            if found:
                self._last_tag_sec = self._now_sec()

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _take_control(self):
        if self._current_pose is None:
            self.get_logger().warn('[PL] No pose yet — deferring takeover')
            self._confirm_count = 0
            return
        active = self._active_tag_id()
        self.get_logger().warn(
            f'[PL] Tag {active} confirmed — taking control from controller')
        self._pl_state      = _State.CENTERING
        self._hold_pose     = self._current_pose
        self._last_tag_sec  = self._now_sec()
        self._confirm_count = 0
        self._pub_takeover(True)

    def _release_control(self):
        self._pl_state      = _State.IDLE
        self._confirm_count = 0
        self._handoff_sent  = False
        self._pub_takeover(False)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        if self._pl_state == _State.IDLE:
            return
        if self._pl_state == _State.CENTERING:
            self._centering_step()
        elif self._pl_state == _State.HANDOFF:
            self._handoff_step()

    def _centering_step(self):
        if self._current_pose is None:
            return

        cur_x = self._current_pose.pose.position.x
        cur_y = self._current_pose.pose.position.y
        cur_z = self._current_pose.pose.position.z

        # Hand off once we're low enough
        if cur_z <= self._final_alt:
            self.get_logger().info(
                f'[PL] Altitude {cur_z:.2f} m <= {self._final_alt} m — HANDOFF')
            self._pl_state = _State.HANDOFF
            return

        # Select tag based on current altitude, log when switching
        active    = self._active_tag_id()
        tag_frame = f'tag{active}'
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))

            tag_x = tf.transform.translation.x  # ENU East
            tag_y = tf.transform.translation.y  # ENU North

            # Lateral correction: move toward tag, descend one step
            new_x = cur_x + (tag_x - cur_x) * self._gain
            new_y = cur_y + (tag_y - cur_y) * self._gain
            new_z = max(cur_z - self._d_step, self._final_alt)

            self._hold_pose    = self._make_setpoint(new_x, new_y, new_z)
            self._last_tag_sec = self._now_sec()
            self._setpoint_pub.publish(self._hold_pose)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Tag not visible — hold position, stop descending
            elapsed = self._now_sec() - self._last_tag_sec
            if elapsed > self._loss_to:
                self.get_logger().warn(
                    f'[PL] Tag lost {elapsed:.1f} s — holding position',
                    throttle_duration_sec=2.0)
            if self._hold_pose is not None:
                # Refresh timestamp so MAVROS doesn't reject a stale setpoint
                self._hold_pose.header.stamp = self.get_clock().now().to_msg()
                self._setpoint_pub.publish(self._hold_pose)

    def _handoff_step(self):
        if self._handoff_sent:
            return
        self._handoff_sent = True

        if self._mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'LAND'
            self._mode_client.call_async(req)
            self.get_logger().info('[PL] LAND mode requested — releasing control')
        else:
            self.get_logger().warn('[PL] set_mode service not ready — releasing anyway')

        self._release_control()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pub_takeover(self, value: bool):
        msg = Bool()
        msg.data = value
        self._takeover_pub.publish(msg)

    def _make_setpoint(self, x, y, z) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        if self._current_pose is not None:
            msg.pose.orientation = self._current_pose.pose.orientation
        else:
            msg.pose.orientation.w = 1.0
        return msg

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
