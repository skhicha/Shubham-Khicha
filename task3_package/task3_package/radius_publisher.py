import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class RadiusPublisher(Node):

    def __init__(self):
        super().__init__('radius_publisher')
        self.publisher_ = self.create_publisher(Float64, '/radius', 10)
        timer_period = 1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.radius = 1.0

    def timer_callback(self):
        msg = Float64()
        msg.data = self.radius
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RadiusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()