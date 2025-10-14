import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, ExtendedState, SysStatus
from diagnostic_msgs.msg import DiagnosticArray

class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')

        # Subscriptions to relevant system topics
        self.create_subscription(State, '/smaug2/state', self.state_callback, 10)
        self.create_subscription(ExtendedState, '/smaug2/extended_state', self.extended_state_callback, 10)
        self.create_subscription(SysStatus, '/smaug2/sys_status', self.sys_status_callback, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)

        # Store latest values
        self.current_state = None
        self.extended_state = None
        self.battery_voltage = None
        self.last_diagnostics = []

        self.get_logger().info("StatusNode initialized. Monitoring drone system state and health...")

    # ---------------- CALLBACKS ----------------

    def state_callback(self, msg: State):
        """Drone connection and mode information"""
        self.current_state = msg
        self.get_logger().info(
            f"[STATE] Connected: {msg.connected}, Armed: {msg.armed}, Mode: {msg.mode}"
        )

    def extended_state_callback(self, msg: ExtendedState):
        """Extended information like landing state"""
        self.extended_state = msg
        self.get_logger().info(
            f"[EXTENDED] Landed State: {msg.landed_state}, VTOL State: {msg.vtol_state}"
        )

    def sys_status_callback(self, msg: SysStatus):
        """System voltage, load, errors, etc."""
        voltage = msg.voltage_battery / 1000.0  # Convert from mV
        self.battery_voltage = voltage
        errors = msg.errors_count1 + msg.errors_count2 + msg.errors_count3 + msg.errors_count4
        self.get_logger().info(
            f"[SYS_STATUS] Voltage: {voltage:.2f}V | Errors: {errors}"
        )

    def diagnostics_callback(self, msg: DiagnosticArray):
        """General diagnostics information"""
        self.last_diagnostics = msg.status
        summary = ", ".join([s.name for s in msg.status if s.level > 0])
        if summary:
            self.get_logger().warn(f"[DIAGNOSTICS WARNING] Issues detected: {summary}")
        else:
            self.get_logger().info("[DIAGNOSTICS] All systems nominal.")

# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("StatusNode shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
