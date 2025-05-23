import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Constants
NAMESPACE = ''  # Namespace, if any (empty string means no namespace)
DOMAIN_ID = 0  # Domain ID for ROS 2 communication
TOPIC = '/joint_command'  # Topic to send joint commands


# Joint names and positions
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
POSITIONS = [  # Array of arrays specifying positions for the robot
    [0.0, 0.5, 1.0, 1.5, 2.0, 2.5],  # Position set 1
    [-0.5, -1.0, -1.5, -2.0, -2.5, -3.0],  # Position set 2
    [1.0, 0.0, -1.0, 1.0, 0.0, -1.0]  # Position set 3
]

class JointCommandPublisher(Node):
    """
    A ROS 2 node that publishes joint commands to a robot.
    """

    def __init__(self):
        super().__init__('joint_command_publisher', namespace=NAMESPACE)
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            TOPIC,
            10  # Queue size
        )
        self.timer = self.create_timer(5.0, self.publish_joint_commands)  # Timer to publish every 5 seconds
        self.position_index = 0  # Tracks the current position set to publish

        self.get_logger().info(f'Initialized JointCommandPublisher node in namespace: {NAMESPACE}')

    def publish_joint_commands(self):
        """
        Publishes joint commands to the defined topic at regular intervals.
        """
        if self.position_index >= len(POSITIONS):
            self.position_index = 0  # Reset to the first position set

        # Create the message
        message = Float64MultiArray()
        message.data = POSITIONS[self.position_index]

        # Log and publish the message
        self.get_logger().info(f'Publishing joint positions: {message.data}')
        self.publisher_.publish(message)

        # Increment the position index
        self.position_index += 1

def main(args=None):

    # Set the domain ID
    import os
    # Check for SimulatorApp instance (Isaac Sim)
    if 'OMNI_APP' in os.environ:
        print("The script is running inside Isaac Sim. (OMNI_APP set)")
    else:
        print("The script is not running inside Isaac Sim. (OMNI_APP not set)")
    try:
        from omni.isaac.kit import SimulatorApp
        if SimulatorApp._instance is not None:
            print("Detected SimulatorApp instance. Exiting ROS 2 publisher script.")
            return
    except ImportError:
        print("Warning: omni.isaac.kit not found. No Omniverse-related libraries available.")
    
    os.environ['ROS_DOMAIN_ID'] = str(DOMAIN_ID)

    # Initialize the ROS 2 Python client library
    #rclpy.init(args=args)

    # Create and spin the node
    joint_command_publisher = JointCommandPublisher()
    try:
        #rclpy.spin(joint_command_publisher)
        pass
    except KeyboardInterrupt:
        pass  # Allow clean shutdown on Ctrl+C
    finally:
        # Destroy the node and shut down ROS
        joint_command_publisher.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
