import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion


class MoveToGoal(Node):
    """
    A ROS2 node that controls a drone to fly to a target (x, y, z) coordinate
    and hover there.

    This class subscribes to odometry and IMU data to track the drone's position
    and orientation. It also listens for a control relinquish message to determine
    whether it should take control of movement. 

    The node:
    1. Takes off and climbs to the target altitude.
    2. Moves horizontally to the target (x, y) position.
    3. Hovers in place once the goal is reached.

    Velocity commands are published to /cmd_vel.
    """

    def __init__(self):
        super().__init__("move_to_goal")

        # Publisher to send velocity commands
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers for odometry and IMU data
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)
        self.ctrl_sub = self.create_subscription(Bool, "ctrl_relinq", self.ctrl_relinq_callback, 1)

        # Drone’s current state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # Control flag
        self.has_control = False

        # Target position (example values — replace with actual GPS/ENU coordinates)
        self.goal_x = 5.0     # target east position
        self.goal_y = -3.0    # target north position
        self.goal_z = 2.5     # target altitude (meters)
        self.goal_yaw = 0.0   # desired final heading (radians)

        # Drone control state machine
        self.state = "ASCEND"

        # Timer for main control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    # -------------------- Callbacks --------------------

    def odom_callback(self, msg: Odometry) -> None:
        """Update drone’s current position from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def imu_callback(self, msg: Imu) -> None:
        """Extract yaw orientation from IMU quaternion."""
        quat = msg.orientation
        (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.yaw = yaw

    def ctrl_relinq_callback(self, msg: Bool) -> None:
        """Enable or disable drone control."""
        self.has_control = msg.data
        if self.has_control:
            self.get_logger().info("Drone control ENABLED")
        else:
            self.get_logger().info("Drone control DISABLED")

    # -------------------- Main Control Loop --------------------

    def control_loop(self) -> None:
        """Main control loop for drone motion."""
        if not self.has_control:
            return

        cmd = Twist()

        # Compute positional errors
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dz = self.goal_z - self.z
        distance_xy = math.sqrt(dx ** 2 + dy ** 2)
        distance_3d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # Compute desired heading toward goal
        target_yaw = math.atan2(dy, dx)
        yaw_error = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi

        # State machine
        if self.state == "ASCEND":
            # Climb to the desired altitude first
            if abs(dz) > 0.1:
                cmd.linear.z = 0.4 * dz  # proportional altitude control
                self.get_logger().info(f"Ascending: z={self.z:.2f}, target={self.goal_z:.2f}")
            else:
                cmd.linear.z = 0.0
                self.state = "MOVE_TO_GOAL"

        elif self.state == "MOVE_TO_GOAL":
            # Move horizontally towards goal position
            if distance_xy > 0.2:
                cmd.linear.x = 0.4 * dx
                cmd.linear.y = 0.4 * dy
                cmd.linear.z = 0.2 * dz  # minor altitude adjustment
                cmd.angular.z = 0.3 * yaw_error  # yaw alignment
                self.get_logger().info(
                    f"Moving: (x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}) "
                    f"→ (gx={self.goal_x:.2f}, gy={self.goal_y:.2f}, gz={self.goal_z:.2f})"
                )
            else:
                cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0
                self.state = "HOVER"

        elif self.state == "HOVER":
            # Maintain position by sending zero velocity
            cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.z = 0.0
            self.get_logger().info("Hovering at goal position.")

        # Publish velocity command
        self.publisher_.publish(cmd)


# -------------------- Entry Point --------------------

def main(args=None):
    """
    Initializes and runs the MoveToGoal node.
    """
    rclpy.init(args=args)   # Initialize ROS2
    node = MoveToGoal()     # Create node instance
    rclpy.spin(node)        # Keep node running
    node.destroy_node()     # Cleanup before shutdown
    rclpy.shutdown()        # Shutdown ROS2


if __name__ == "__main__":
    main()  # Execute the main function