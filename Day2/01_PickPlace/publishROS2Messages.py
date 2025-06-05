import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

#Helper Methods 
def log_warning(msg):
    try:
        import carb
        carb.log_warn(msg)
    except ImportError:
        print(f"WARNING: {msg}")

def log_error(msg):
    try:
        import carb
        carb.log_error(msg)
    except ImportError:
        print(f"ERROR: {msg}")

# Constants
NAMESPACE = ''  # Namespace, if any (empty string means no namespace)
DOMAIN_ID = 0  # Domain ID for ROS 2 communication
TOPIC = '/joint_command'  # Topic to send joint commands
DELAY = 1.5  # Delay in seconds between messages


# Joint names and positions (gripper 0..closed, 1..open)
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'gripper_state'] 
POSITIONS = [  # Array of arrays specifying positions for the robot
    [0.68, -1.36, 1.75, 0.0, 1.19, 0.0, 1.0],  # Hover over right container
    [0.59, -0.93, 1.56, 0.0, 0.939, 0.0, 1.0],  # Target Red cube
    [0.59, -0.93, 1.56, 0.0, 0.939, 0.0, 0.0],  # Attach Red cube
    [0.68, -1.36, 1.75, 0.0, 1.19, 0.0, 0.0],  # Hover over right container
    [-0.52, -1.36, 1.75, 0.0, 1.19, 0.0, 0.0],  # Hover over left container
    [-0.52, -1.16, 1.92, 0.0, 0.79, 0.0, 0.0],  # Lower Red cube
    [-0.52, -1.16, 1.92, 0.0, 0.79, 0.0, 1.0],  # Drop Red cube
    [-0.52, -1.36, 1.75, 0.0, 1.19, 0.0, 1.0],  # Hover over left container
    [0.68, -1.36, 1.75, 0.0, 1.19, 0.0, 1.0],  # Hover over right container
    [0.70, -1.08, 1.82, 0.0, 0.83, 0.0, 1.0],  # Target Blue cube
    [0.70, -1.08, 1.82, 0.0, 0.83, 0.0, 0.0],  # Attach Blue cube
    [0.68, -1.36, 1.75, 0.0, 1.19, 0.0, 0.0],  # Hover over right container
    [-0.52, -1.36, 1.75, 0.0, 1.19, 0.0, 0.0],  # Hover over left container
    [-0.52, -1.23, 1.88, 0.0, 0.91, 0.0, 0.0],  # Lower Blue cube
    [-0.52, -1.23, 1.88, 0.0, 0.91, 0.0, 1.0],  # Drop Blue cube
    [-0.52, -1.36, 1.75, 0.0, 1.19, 0.0, 1.0],  # Hover over left container
    [0.68, -1.36, 1.75, 0.0, 1.19, 0.0, 1.0],  # Hover over right container
    [0.879, -1.22, 2.04, 0.0, 0.748, 0.0, 1.0],  # Target Green cube
    [0.879, -1.22, 2.04, 0.0, 0.748, 0.0, 0.0],  # Attach Green cube
    [0.68, -1.36, 1.75, 0.0, 1.19, 0.0, 0.0],  # Hover over right container
    [-0.52, -1.36, 1.75, 0.0, 1.19, 0.0, 0.0],  # Hover over left container
    [-0.52, -1.28, 1.85, 0.0, 0.99, 0.0, 0.0],  # Lower Green cube
    [-0.52, -1.28, 1.85, 0.0, 0.99, 0.0, 1.0],  # Drop Green cube
    [-0.52, -1.36, 1.75, 0.0, 1.19, 0.0, 1.0],  # Hover over left container
]

    #[-0.52, -1.16, 1.92, 0.0, 0.81, 0.0, 0.0],  # Lower Red cube
    #[-0.52, -1.23, 1.88, 0.0, 0.91, 0.0, 0.0],  # Lower Blue cube
    #[-0.52, -1.28, 1.85, 0.0, 0.99, 0.0, 0.0],  # Lower Green cube

class JointCommandPublisher(Node):
    """
    A ROS 2 node that publishes joint commands to a robot.
    """

    def __init__(self):
        super().__init__('joint_command_publisher', namespace=NAMESPACE)
        self.publisher_ = self.create_publisher(
            JointState,
            TOPIC,
            10  # Queue size
        )
        self.timer = self.create_timer(DELAY, self.publish_joint_commands)  # Timer to publish every 5 seconds
        self.position_index = 0  # Tracks the current position set to publish

        self.get_logger().info(f'Initialized JointCommandPublisher node in namespace: {NAMESPACE}')

    def publish_joint_commands(self):
        """
        Publishes joint commands to the defined topic at regular intervals.
        """
        if self.position_index >= len(POSITIONS):
            self.position_index = 0  # Reset to the first position set

        # Create the message
        message = JointState()
        message.name = JOINT_NAMES
        message.position = POSITIONS[self.position_index]

        # Log and publish the message
        self.get_logger().info(f'Publishing joint positions: {message.position}')
        self.publisher_.publish(message)

        # Increment the position index
        self.position_index += 1

def main(args=None):

    # Set the domain ID
    import os
    try:
        import omni.kit.app
        try:
            app = omni.kit.app.get_app()
            if app is not None:
                log_error("Detected SimulatorApp instance. Exiting ROS 2 publisher script. (Do not run script from within App)")
                return
        except RuntimeError:
            # This happens if the Kit app is not running (e.g., in python.sh)
            pass
    except ImportError:
        log_warning("omni.isaac.kit not found. No Omniverse-related libraries available.")
    
    os.environ['ROS_DOMAIN_ID'] = str(DOMAIN_ID)

    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create and spin the node
    joint_command_publisher = JointCommandPublisher()
    try:
        rclpy.spin(joint_command_publisher)
        pass
    except KeyboardInterrupt:
        # Allow clean shutdown on Ctrl+C
        joint_command_publisher.get_logger.info("KeyboardInterrupt received. Shutting down...")
    finally:
        # Destroy the node and shut down ROS
        joint_command_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
