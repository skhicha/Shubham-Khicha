import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Node2(Node):

    def __init__(self):
        super().__init__('node2')
        self.publisher_ = self.create_publisher(String, '/world', 10)
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'World!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = Node2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()