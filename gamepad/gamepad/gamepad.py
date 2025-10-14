#!/usr/bin/env python3
# The above line is a shebang, which tells the system to run this script using Python 3.

# Import necessary ROS 2 libraries
import rclpy  # ROS 2 client library for Python
from std_msgs.msg import Bool
from rclpy.node import Node  # Base class for creating ROS 2 nodes

# Import message types
from sensor_msgs.msg import Joy  # Message type for joystick (gamepad) inputs
from geometry_msgs.msg import Twist  # Message type for velocity commands

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import SetInitialPose
import math

class Gamepad(Node):
    """
    A ROS 2 Node that converts joystick (gamepad) inputs into velocity commands
    for a robot. It subscribes to the 'joy' topic and publishes Twist messages
    to the 'cmd_vel' topic.
    """

    
    def __init__(self):
        """
        Constructor: Initializes the gamepad node.
        - Subscribes to the 'joy' topic to receive joystick inputs.
        - Publishes to the 'cmd_vel' topic to send velocity commands.
        """
        # TODO: Initialize the node with the name 'gamepad'
        super().__init__('gamepad')

        

        # TODO: Create a service client to call the '/set_pose' service
        # - Service type: 'SetInitialPose'
        # - Service name: '/set_pose'
        self.reset_pose_client = self.create_client(SetInitialPose, '/set_pose')

        # TODO: Create a subscriber to the 'joy' topic (gamepad inputs)
        # - This listens for messages of type Joy.
        # - It calls the `joy_callback` function whenever a new message arrives.
        # - Queue size of 10 will buffer up to 10 messages before discarding old ones.
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # TODO: Create a publisher to send velocity commands to the 'cmd_vel' topic.
        # - This sends messages of type Twist.
        # - Queue size of 10 helps manage message buffering.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Log a message indicating that the node has started successfully
        self.get_logger().info("Joy to cmd_vel node started!")
        # Flag to track control status (default: True)
        self.has_control = True

        # Create a publisher for control relinquishment messages.
        # - Publishes to the 'ctrl_relinq' topic.
        # - Uses Bool messages to indicate control status.
        # - Queue size of 1 ensures only the latest control state is kept.
        self.ctrl_pub = self.create_publisher(Bool, 'ctrl_relinq', 1)

    def joy_callback(self, msg):
        """
        Callback function that processes incoming joystick messages.
        - Extracts axis values from the joystick message.
        - Converts these values into a Twist message (velocity commands).
        - Publishes the Twist message to control the robot.

        Args:
            msg (Joy): The incoming joystick message containing axes and button states.
        """
        # Check if button 3 (Y) is pressed to reset the pose
        if msg.buttons[3]:
            self.send_set_pose_request(0.0, 0.0, 0.0)  # Call the reset_pose service

        # TODO: Create a new Twist message for velocity commands.
        newMessage = Twist()

        # TODO: Map joystick axes to robot velocity:
        # - The left stick (up/down) controls linear speed (forward/backward).
        # - The right stick (left/right) controls angular speed (rotation).
        # Ensure that the joystick axis values are properly scaled to match 
        # the robot's velocity limits:
        # - Max Linear Velocity: 0.22 m/s
        # - Max Angular Velocity: 2.84 rad/s
        newMessage.linear.x = .22*msg.axes[1]
        newMessage.angular.z = 2.84*msg.axes[3]

        # TODO: Publish the velocity command to the '/cmd_vel' topic.
        if self.has_control:
            self.publisher_.publish(newMessage)

        # Create a new Bool message for "control relinquishment status"
        relinquish = Bool()

        # TODO: Check if the RC (Remote Control) has control and button A (Green) is pressed
        # Set control flag to False (relinquish control)
        # Change "control status" to relinquished.
        # Publish the control status
        if msg.buttons[0] and self.has_control:
            self.has_control = False
            relinquish.data = True
            self.ctrl_pub.publish(relinquish)

            # Log status update
            self.get_logger().info("RC has relinquished control.")  

        # TODO: Check if RC does not have control and button B (Red) is pressed
        # Set control flag to True (regain control)
        # Set control status to regained.
        # Publish the control status
        if msg.buttons[1] and self.has_control == False:
            self.has_control = True
            relinquish.data = False
            self.ctrl_pub.publish(relinquish)
            
            # Log status update
            self.get_logger().info("RC has taken over control.")  

        # If control is relinquished, stop further processing
        if not self.has_control:
            return
        
    def send_set_pose_request(self, x:float, y:float, theta:float):
        """
        Calls the 'reset_pose' service to reset the robot's pose.
        """
        # Check if the service is available
        if not self.reset_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service 'reset_pose' not available.")
            return

        # Create a request object 
        request = SetInitialPose.Request()
        request.pose = PoseWithCovarianceStamped()  

        # Set Header
        request.pose.header.frame_id = "map"
        request.pose.header.stamp = self.get_clock().now().to_msg()

        # TODO: Set Position
        self.x = request.pose.pose.pose.position.x
        self.y = request.pose.pose.pose.position.y
        
        # TODO: Convert Yaw (theta) to Quaternion
        # z = sin(theta/2), w = cos(theta/2)
        q = request.pose.pose.pose.orientation
        q.w = math.cos(theta / 2)
        q.z = math.sin(theta / 2)

        # TODO: Set Covariance (required by AMCL, small uncertainty such as [0.1]*36)
        covariance = [0.1] * 36

        # Call the service asynchronously
        future = self.reset_pose_client.call_async(request)

        # Add a callback to handle the service response
        future.add_done_callback(self.reset_pose_done_callback)

    def reset_pose_done_callback(self, future):
        """
        Callback function to handle the response from the 'reset_pose' service.
        """
        try:
            # Check if the service call was successful
            response = future.result()
            if response:
                self.get_logger().info("Pose reset successfully.")
            else:
                self.get_logger().error("Pose reset failed.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    """
    Main function to start the ROS 2 node.
    - Initializes the ROS 2 system.
    - Creates an instance of the Gamepad node.
    - Keeps the node running using `rclpy.spin()`, which listens for messages.
    - Cleans up resources when the node is shut down.
    """
    rclpy.init(args=args)  # Initialize ROS 2
    gamepad = Gamepad()  # Create an instance of the Gamepad node
    rclpy.spin(gamepad)  # Keep the node running and responsive to joystick input

    # Cleanup when the node is shutting down
    gamepad.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown ROS 2

# Run the script if executed directly (not imported as a module)
if __name__ == '__main__':
    main()