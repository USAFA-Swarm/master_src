#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool
 
class ControlToggleNode(Node):
    """
    Monitors flight mode and RC input.
    Publishes a Boolean topic indicating whether autonomous mode is active.
    """
 
    def __init__(self):
        super().__init__('control_toggle_node')
        # Subscribers
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, 10)
 
        # Publisher: tells the rest of the system whether autonomy is active
        self.autonomy_pub = self.create_publisher(Bool, '/has_autonomy', 10)
 
        # Service client for setting flight mode
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
 
        # Internal state
        self.current_mode = "MANUAL"
        self.has_autonomy = False
        self.get_logger().info("Control Toggle Node started — monitoring RC and mode state...")
 
    def state_callback(self, msg: State):

        """Track the current flight mode and publish autonomy status."""
        if msg.mode != self.current_mode:
            self.get_logger().info(f"Flight mode changed: {self.current_mode} → {msg.mode}")
            self.current_mode = msg.mode

        new_state = (msg.mode == "GUIDED")
        if new_state != self.has_autonomy:
            self.has_autonomy = new_state
            self.autonomy_pub.publish(Bool(data=self.has_autonomy))
            self.get_logger().info(f"Autonomy {'ENABLED' if self.has_autonomy else 'DISABLED'}")
 
    def rc_callback(self, msg: RCIn):

        """
        Optionally monitor a specific RC channel (like CH7) to trigger mode switching.
        For example, CH7 high -> switch to GUIDED (autonomy), CH7 low -> switch to MANUAL.
        """

        if len(msg.channels) < 7:
            return
 
        ch7 = msg.channels[6]  # Channel 7 (0-indexed)
        desired_mode = "GUIDED" if ch7 > 1500 else "MANUAL"
 
        if desired_mode != self.current_mode and self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = desired_mode
            self.set_mode_client.call_async(req)
            self.get_logger().info(f"Requested mode change to {desired_mode}")
 
def main(args=None):
    rclpy.init(args=args)
    node = ControlToggleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()

 