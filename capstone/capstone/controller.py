import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from nav2_msgs.srv import SetInitialPose
from visualization_msgs.msg import Marker
import numpy as np

# Hardcoded Values
BEFORE_TAG_DIST = 1.0 # Go straight from here
TAG_DIST = 0.30   # Distance to AprilTag
STOP_DIST = 0.56  # Distance to stop sign
TURN_RATE = 0.6  # Rotation speed in radians per second
MIDDLE_DIST = 0.265 # Distance from middle to wall
DIST_FORWARD = 0.69 # Distance to drive after detecting april tag 2


class Controller(Node):
    """
    A ROS2 node that controls a robot's decision tree.

    This class subscribes to odometry and IMU topics to track the robot's position
    and orientation. The class also subscribes to the apriltag_pose and stop_dist topics
    to track what the robot is seeing through the camera. The robot also listens to the 
    centerline topic to maintain it's position in the center of the maze. It also listens 
    for control relinquishment messages to determine whether it should take control of 
    movement. The node follows a state machine approach 
    1. Move through a maze.
    2. Make decisions at AprilTags/Stop Signs
    3. Stop after finding the treasure.

    Velocity commands are published to the '/cmd_vel' topic to control the robot's movement.
    """
    def __init__(self):
        super().__init__("controller")  # Initialize the ROS2 node with name 'controller'

        # Publisher for sending velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribers to receive odom and imu data
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)

        # Subscriber to receive control status
        self.ctrl_sub = self.create_subscription(Bool, "ctrl_relinq", self.ctrl_relinq_callback, 1)

        # Subscribers to recieve AprilTag and Stop Sign data
        self.april_sub = self.create_subscription(PoseStamped, "apriltag_pose", self.april_callback, 10)
        self.stop_sub = self.create_subscription(Float32, "/stop_dist", self.stop_callback, 10)

        # Subscriber that listens to the center line marker
        self.center_line_sub = self.create_subscription(Marker, "/center_line", self.follow_line, 10)
        self.wall_marker_sub = self.create_subscription(Marker, "/wall_marker", self.follow_line, 10)

        # Variables to store the robot's current position and orientation
        self.x = 0      # Current x-coordinate
        self.y = 0      # Current y-coordinate
        self.yaw = 0    # Current orientation (yaw angle in radians)

        # Variables for previous position to compute displacement
        self.initial_global_x = None
        self.initial_global_y = None
        self.initial_global_yaw = None

        # Variables for local position in the robot's frame
        self.local_x = 0                # Local x-coordinate
        self.local_y = 0                # Local y-coordinate

        # Variables for yaw angle in the robot's frame
        self.prev_yaw   = None          # yaw measured on the previous loop
        self.turn_accum = 0.0           # total rotation (rad)

        # Flag indicating whether this node has control
        self.has_control = False   

        # Initial state of the robot
        self.state = "WALL_FOLLOWING"  

        # Initialize StopSign and AprilTag Variables
        self.stop_dist = None  # Distance to the stop sign
        self.tag_id = None  # ID of the detected AprilTag
        self.april_dist = None  # Distance to the AprilTag
        self.tag_x = None       # x-coordinate of the AprilTag in the robot's frame

        # Variables to store the robot's current orientation
        self.current_orientation = None

        # Flag to indicate if the robot is in the middle of a window
        self.window = False

        # Initialize the interation timer for the STOP_5_SECONDS state
        self.iteration = 0

        # Initialize the cmd_vel variables
        self.cmd = Twist()         # Command message stopping
        self.cmd_follow = Twist()  # Command message for line following
        self.cmd_turn = Twist()    # Command message for turning

        # Timer to run the control loop at a fixed rate (every 0.1 seconds) 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # A service server that will handle '/set_pose' service requests
        # - Service type: 'SetInitialPose'
        # - Service name: '/set_pose'
        # - Callback function: 'self.set_pose_callback' to execute when the service is called
        self.reset_service = self.create_service(SetInitialPose, "set_pose", self.set_pose_callback)

        # Log that the controller node has started
        self.get_logger().info("Controller Node Started")


    def stop_callback(self, msg: Float32) -> None:
        """
        Callback function for handling stop distance messages.
        Updates the distance based on the received message.
        """
        # Extract the StopSign distance from the message
        self.stop_dist = msg.data


    def april_callback(self, msg: PoseStamped) -> None:
        """
        Callback function for handling AprilTag pose messages.
        Updates the tagID and distance based on the received message.
        """
        # Extract the AprilTag data from the message
        self.tag_id = msg.header.frame_id       # AprilTag ID
        self.april_dist = msg.pose.position.z   # Distance to the AprilTag
        self.tag_x = msg.pose.position.x        # x-coordinate of the AprilTag in the robot's frame


    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback function for handling odometry messages.
        Updates the robot's current x and y position.
        """
        # Extract x-coordinate from odometry
        global_x = msg.pose.pose.position.x
        # Extract y-coordinate from odometry
        global_y = msg.pose.pose.position.y

        # Check if this is the first iteration (initial global position not set)
        if self.initial_global_x is None:
            # Store the initial global position as the reference point
            self.initial_global_x = global_x
            self.initial_global_y = global_y
            # Skip processing for the first iteration
            return
        
        # Compute the displacement of the global position from the initial global position
        dx = global_x - self.initial_global_x
        dy = global_y - self.initial_global_y

        # Rotate displacement to align with the initial local frame
        # This step is necessary to ensure the local coordinates are relative to the starting orientation
        self.local_x = dx * math.cos(self.initial_global_yaw) + dy * math.sin(self.initial_global_yaw)
        self.local_y = - (dx * math.sin(self.initial_global_yaw)) + dy * math.cos(self.initial_global_yaw)


    def imu_callback(self, imu_msg: Imu) -> None:
        """
        Callback function for handling control relinquishment messages.
        Updates local yaw relative to the starting orientation.
        """
        q = imu_msg.orientation
        _, _, global_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.initial_global_yaw is None:
            # Store the initial yaw as a reference
            self.initial_global_yaw = global_yaw
            return  # Skip first iteration

        # Compute local yaw relative to the initial global yaw
        self.local_yaw = global_yaw - self.initial_global_yaw

        # Update the current orientation
        self.current_orientation = imu_msg.orientation


    def ctrl_relinq_callback(self, relinq_msg: Bool) -> None:
        """
        Callback function for handling control relinquishment messages.
        Updates the control status based on received messages.
        """
        # Update control flag
        self.has_control = relinq_msg.data

        # Log the control status
        if self.has_control:
            self.get_logger().info("TreasureHunt has taken control")
        else:
            self.get_logger().info("TreasureHunt has lost control")


    def set_pose_callback(self, request: SetInitialPose.Request, response: Empty.Response) -> Empty.Response:
        """
        Resets the local position coordinates to zero and clears the previous position coordinates.
        """
        # Reset the local position coordinates
        self.local_x = request.pose.pose.pose.position.x
        self.local_y = request.pose.pose.pose.position.y

        # Convert Quaternion to Euler Angles (Extract Yaw)
        q = request.pose.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, self.local_yaw = euler_from_quaternion(quaternion)

        # Reset the state
        self.state = "WALL_FOLLOWING"

        # Clear the previous position coordinates
        # This ensures that any previous position data is discarded
        self.initial_global_x = None
        self.initial_global_y = None
        self.initial_global_yaw = None

        # Log a message to indicate the local position has been reset
        self.get_logger().info(f"Local pose set to x={self.local_x}, y={self.local_y}, yaw={self.local_yaw}.")

        # Return the response to the service caller
        return response
    

    def follow_line(self, center_line: Marker) -> None:
        """
        Callback function for the center line marker.
        Computes the robot's alignment with the line and adjusts its movement to follow it.
        """
        # Proceed only if the robot is in the WALL_FOLLOWING state
        if self.state != "WALL_FOLLOWING":
            return

        # Proceed only if the robot has control
        if not self.has_control:
            return

        # Wait until we have IMU data
        if self.current_orientation is None:
            return  

        # Not enough points to follow a line
        if len(center_line.points) < 2:
            return  
        
        # Extract the first and last points of the center line
        start_point = center_line.points[0]
        end_point = center_line.points[-1]

        # Compute the slope of the line and its angle relative to the x-axis.
        # The angle of the line relative to the x-axis is the arctangent of the slope.
        line_dx = end_point.x - start_point.x # Change in x (x2 - x1)
        line_dy = end_point.y - start_point.y # Change in y (y2 - y1)
        
        m = line_dy / line_dx # slope (m)
        k = start_point.y - m * start_point.x # y-intercept (k)
        

        # Convert the line equation y = mx + k into the standard form ax + by + c = 0
        # line is x = constant
        if abs(line_dx) < 1e-6:              
            a, b, c = 1.0, 0.0, -start_point.x
            line_angle = np.pi / 2           # 90° in robot frame
        # line is y = mx + k
        else:            
            a, b, c = -m, 1.0, -k
            line_angle = np.arctan2(line_dy, line_dx)

        # Compute the perpendicular distance (d) from the robot to the line
        distance_error = c / np.hypot(a, b)

        # Compute the angle error between the robot's heading and the line
        angle_error = np.arctan2(np.sin(line_angle), np.cos(line_angle))  # wrap to [-π, π]

        # Controller gains
        kh = 1.5  # Heading controller gain (adjusts rotation based on angle error)
        kd = 3.0  # Distance controller gain (adjusts rotation based on distance error)

        # Adjust rotation speed based on angle error and distance error
        gamma = -kd*distance_error + kh * angle_error

        # Debug line following errors
        self.get_logger().debug(
            f"distance error: {distance_error}, angle error: {angle_error}, gamma={gamma}"
        )

        # Publish the velocity command
        # If the distance error is greater than the middle distance, there's an evil window
        if (abs(distance_error) > MIDDLE_DIST):
            self.cmd_follow.linear.x = 0.0      # Stop the forward motion
            self.cmd_follow.angular.z = 0.0     # Stop the rotation
            self.window = True                  # Evil window detected
        # If the distance error is less than the middle distance, we are still within range of pathway
        else:
            self.cmd_follow.linear.x = 0.15     # Move forward at a constant speed
            self.cmd_follow.angular.z = gamma   # Rotate to align with the line
            self.window = False                 # No evil window detected


    def control_loop(self) -> None:
        """
        Main control loop that executes periodically.
        Controls the robot's movement towards the Treasure.
        """
        # Exit if this node does not have control
        if not self.has_control:
            return  

        # Log current state
        self.get_logger().info(f"Current State: {self.state}")


        # State Machine Decision Logic
        if self.state == "WALL_FOLLOWING":
            """
            Follow the center line between the walls. Make decisions 
            based on the detected AprilTag or stop sign.
            """
            # For all AprilTags aside from ID 4, enter the BEFORE_APRILTAG approach state
            if (self.tag_id is not None and self.tag_id is not '4' and self.april_dist < BEFORE_TAG_DIST):
                self.state = "BEFORE_APRILTAG"
                self.cmd_follow.linear.x = 0.15
                self.cmd_follow.angular.z = 0.0 # Stop the rotation
            # If a stop sign is detected, enter the STOP_DETECTED state
            elif (self.stop_dist is not None and self.stop_dist < STOP_DIST):
                self.state = "STOP_DETECTED"
                self.cmd_follow.linear.x = 0.0  # Stop the forward motion
                self.cmd_follow.angular.z = 0.0 # Stop the rotation
            # If an evil window is detected, stop rotation and remain going forward
            elif (self.window is True):
                self.cmd_follow.linear.x = 0.15
                self.cmd_follow.angular.z = 0.0
                self.state = "WALL_FOLLOWING"
            # If no tag, stop sign, or evil window is detected, continue following the wall
            else:
                self.state = "WALL_FOLLOWING"

            # Publish the wall following velocity command
            self.publisher_.publish(self.cmd_follow)

        elif self.state == "BEFORE_APRILTAG":
            """
            Approach the center of the AprilTag until within TAG_DIST.
            Stop forward movement and rotation to make decisions based 
            on the ID of the detected AprilTag.
            """
            # TagID 0
            if (self.tag_id == '0' and self.april_dist < TAG_DIST):
                self.state = "RIGHT_270"
                self.cmd_follow.linear.x = 0.0
                self.cmd_follow.angular.z = 0.0
            # TagID 1
            elif (self.tag_id == '1' and self.april_dist < TAG_DIST):
                self.state = "STOP_5_SECONDS"
                self.cmd_follow.linear.x = 0.0
                self.cmd_follow.angular.z = 0.0
            # TagID 2
            elif (self.tag_id == '2' and self.april_dist < TAG_DIST):
                self.state = "RIGHT_90"
                self.cmd_follow.linear.x = 0.0
                self.cmd_follow.angular.z = 0.0
            # TagID 3
            elif (self.tag_id == '3' and self.april_dist < TAG_DIST):
                self.state = "LEFT_90"
                self.cmd_follow.linear.x = 0.0
                self.cmd_follow.angular.z = 0.0
            # If no tag is detected, continue moving straight forward
            else:
                self.cmd_follow.linear.x = 0.15
                self.cmd_follow.angular.z = 0.0
                self.state = "BEFORE_APRILTAG"

            # Move forward with wall follower cmd
            K = 4.0 # Proportional gain for rotation
            # If not changing state
            if self.state == "BEFORE_APRILTAG":
                # If tag still there
                if self.april_dist is not None and self.april_dist < BEFORE_TAG_DIST:
                    self.cmd_follow.linear.x = 0.15
                    self.cmd_follow.angular.z = -K * self.tag_x     # Rotate to align with the center of tag
                # If tag not there or too far
                else:
                    self.cmd_follow.linear.x = 0.15
                    self.cmd_follow.angular.z = 0.0
            
            # Publish the velocity command
            self.publisher_.publish(self.cmd_follow)

        elif self.state == "RIGHT_270":
            """
            Rotate -270 degrees (3pi/2 rad) clockwise.
            We integrate the change in yaw so angle wrap-around cannot fool us.
            """
            # IMU not ready yet
            if self.local_yaw is None:          
                return

            # First pass inside this state
            if self.prev_yaw is None:
                self.prev_yaw = self.local_yaw      # Save current as previous yaw
                self.turn_accum = 0.0               # Reset the accumulated turn

            # Incremental change in yaw since last loop
            delta = self.local_yaw - self.prev_yaw

            # Correct for wrap-around of yaw to ensure everthing is in the range [-pi, pi]
            # If larger than pi
            if delta > math.pi: 
                delta -= 2*math.pi
            # If less than -pi
            elif delta < -math.pi: 
                delta += 2*math.pi

            # Update the accumulated yaw of the turn
            self.turn_accum += delta

            # Save the current yaw as the previous yaw for the next iteration
            self.prev_yaw = self.local_yaw

            # Keep turning until accumulated turn reaches 270 degrees
            if abs(self.turn_accum) < math.radians(270) - 0.02:   # 0.02 rad about 1 degree
                self.cmd_turn.linear.x = 0.0
                self.cmd_turn.angular.z = -TURN_RATE              # negative = turn right
            # If finished with the turn, reset variables and resume wall following
            else:
                self.cmd_turn = Twist()
                self.state = "WALL_FOLLOWING"
                self.tag_id = None # Clear the tag ID
                self.april_dist = None # Clear the distance to the AprilTag
                self.prev_yaw = None
                self.turn_accum = 0.0

            # Publish the turn velocity command
            self.publisher_.publish(self.cmd_turn)

        elif self.state == "STOP_5_SECONDS":
            """
            Stop for 5 seconds.
            """
            # Stop the robot movement
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

            # 50 iterations at 0.1 seconds each = 5 seconds
            if self.iteration == 50:    # Exit state after 50 iterations
                self.state = "LEFT_90"
                self.iteration = 0 # Reset the iteration counter
            else:                       # Remain in state and increment the iteration counter
                self.iteration += 1

            # Publish the stop velocity command
            self.publisher_.publish(self.cmd)

        elif self.state == "LEFT_90":
            """
            Rotate +90 degrees (pi/2 rad) counter-clockwise.
            """
            # IMU not ready yet
            if self.local_yaw is None:
                return

            # First pass inside this state
            if self.prev_yaw is None:
                self.prev_yaw   = self.local_yaw    # Save current as previous yaw
                self.turn_accum = 0.0               # Reset the accumulated turn amount

            # Incremental change in yaw
            delta = self.local_yaw - self.prev_yaw

            # Correct for wrap-around of yaw to ensure everthing is in the range [-pi, pi]
            # If larger than pi
            if delta >  math.pi: 
                delta -= 2*math.pi
            # If less than -pi
            elif delta < -math.pi: 
                delta += 2*math.pi

            # Update the accumulated yaw of the turn
            self.turn_accum += delta

            # Save the current yaw as the previous yaw for the next iteration
            self.prev_yaw    = self.local_yaw

            # Compute the angle error to the target angle
            angle_error = math.radians(90)-self.turn_accum
    
            # If not finished turning, keep turning
            if self.turn_accum < math.radians(90) - 0.02:     # keep turning left
                self.cmd_turn.angular.z =  TURN_RATE *angle_error   # Proportional controller for turn rate
            # If finished with the turn, reset variables
            else:
                # If tag 2 is detected, center on the tag and skip wall following
                if self.tag_id == '2':
                    self.cmd_turn = Twist()
                    self.state = "BEFORE_APRILTAG"
                    self.prev_yaw = None
                    self.turn_accum = 0.0
                # Otherwise, resume wall following
                else:
                    self.cmd_turn = Twist()
                    self.state = "WALL_FOLLOWING"
                    self.tag_id = None
                    self.april_dist = None
                    self.prev_yaw = None
                    self.turn_accum = 0.0

            # Publish the turn velocity command
            self.publisher_.publish(self.cmd_turn)

        elif self.state == "RIGHT_90":
            """
            Rotate –90 degrees (–pi/2 rad) clockwise.
            """
            # IMU not ready yet
            if self.local_yaw is None:
                return

            # First pass inside this state
            if self.prev_yaw is None:
                self.prev_yaw = self.local_yaw      # Save current as previous yaw
                self.turn_accum = 0.0               # Reset the accumulated turn amount

            # Incremental change in yaw
            delta = self.local_yaw - self.prev_yaw

            # Correct for wrap-around of yaw to ensure everthing is in the range [-pi, pi]
            # If larger than pi
            if delta >  math.pi: 
                delta -= 2*math.pi
            # If less than -pi
            elif delta < -math.pi: 
                delta += 2*math.pi

            # Update the accumulated yaw of the turn
            self.turn_accum += delta

            # Save the current yaw as the previous yaw for the next iteration
            self.prev_yaw = self.local_yaw

            # If not finished turning, keep turning
            if self.turn_accum > -math.radians(90) + 0.02:
                self.cmd_turn.angular.z = -TURN_RATE
            # If finished with the turn, reset variables
            else:
                self.cmd_turn = Twist()
                self.tag_id = None
                self.april_dist = None
                self.state = "WALL_FOLLOWING"
                self.prev_yaw = None
                self.turn_accum = 0.0

            # Publish the turn velocity command
            self.publisher_.publish(self.cmd_turn)

        elif self.state == "STOP_DETECTED":
            """
            Stop when a stop sign is detected.
            """
            # Stop the robot movement
            self.cmd_follow.linear.x = 0.0
            self.cmd_follow.angular.z = 0.0

            # Clear the AprilTag data
            self.tag_id = None
            self.april_dist = None

            # Enter the LEFT_TURN_LOOK state
            self.state = "LEFT_TURN_LOOK"

            # Publish the stop velocity command
            self.publisher_.publish(self.cmd_follow)

        elif self.state == "LEFT_TURN_LOOK":
            """
            Rotate 90 degrees to the left (counter clockwise) and look for Tag_ID = 4.
            """
            # If Tag ID 4 is detected, stop movement and enter the FINISHED state
            if self.tag_id == '4':
                self.cmd_turn.angular.z = 0.0
                self.state = "FINISHED"
            # If no tag is detected, continue turning left
            else:
                self.cmd_turn.angular.z = TURN_RATE/4

            # Publish the turn velocity command
            self.publisher_.publish(self.cmd_turn)

        elif self.state == "FINISHED":
            """
            Robot has reached the treasure. It is finished.
            """
            # Log the treasure found message and the coordinates
            self.get_logger().info(f"Treasure found at x: {self.local_x}, y: {self.local_y}")

            # Stop the robot movement
            self.cmd_turn = Twist()

            # Publish the stop velocity command
            self.publisher_.publish(self.cmd_turn)

        
def main(args=None):
    """
    Main entry point of the node. Initializes and runs the MoveToGoal node.
    """
    rclpy.init(args=args)  # Initialize ROS2
    node = Controller()  # Create node instance
    rclpy.spin(node)  # Keep node running
    node.destroy_node()  # Cleanup before shutdown
    rclpy.shutdown()  # Shutdown ROS2

if __name__ == "__main__":
    main()  # Execute the script