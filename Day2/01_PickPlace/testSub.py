import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState  # Change this to your message type

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,            # Message type
            '/joint_command',      # Topic name
            self.listener_callback,# Callback function
            10                     # QoS history depth
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.position}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    try:
        print("Listening for messages...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow clean shutdown on Ctrl+C
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        # Destroy the node and shut down ROS
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
