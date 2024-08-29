import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class InfinityTwistNode(Node):
    def __init__(self):
        super().__init__('infinity_twist_node')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.radius = 2.0
        self.steps = 100  
        self.linear_velocity = 0.2  

        self.angle = 0.0  
        self.circle = 1 

        self.create_timer(0.1, self.move_infinity)

    def move_infinity(self):
        twist = Twist()

        twist.linear.x = self.linear_velocity

        if self.circle == 1:
            twist.angular.z = self.linear_velocity / self.radius
        else:
            twist.angular.z = -self.linear_velocity / self.radius

        self.publisher.publish(twist)

        self.angle += self.linear_velocity / self.radius / 10

        if self.angle >= 2 * math.pi:
            self.angle = 0.0
            self.circle = 2 if self.circle == 1 else 1
            self.get_logger().info(f"Switching to circle {self.circle}")

def main(args=None):
    rclpy.init(args=args)
    node = InfinityTwistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
