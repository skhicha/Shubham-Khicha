import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Node3(Node):

    def __init__(self):
        super().__init__('node3')
        self.hello_msg = None
        self.world_msg = None
        self.publisher_ = self.create_publisher(
            String,
            '/helloworld', 
            10)
        self.hello_sub = self.create_subscription(
            String,
            '/hello',
            self.hello_callback,
            10)
        self.world_sub = self.create_subscription(
            String,
            '/world',
            self.world_callback,
            10)

    def hello_callback(self, msg):
        self.hello_msg = msg.data
        self.publish_combined_message()
    
    def world_callback(self, msg):
        self.world_msg = msg.data
        self.publish_combined_message()
    
    def publish_combined_message(self):
        if self.hello_msg and self.world_msg:
            combined_msg = String()
            combined_msg.data = f'{self.hello_msg}{self.world_msg}'
            self.publisher_.publish(combined_msg)
            self.get_logger().info(f'Publishing: {combined_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Node3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()