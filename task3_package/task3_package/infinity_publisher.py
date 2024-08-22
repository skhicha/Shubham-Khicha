import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist
from math import sin, cos, pi

class InfinityPublisher(Node):
    def __init__(self):
        super().__init__('infinity_publisher')

        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport_absolute service...')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer_period = 0.1  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.time_elapsed = 0.0
        self.radius = 1.0
        self.angular_velocity = 0.0

    def cmd_vel_callback(self, msg):
        self.radius = msg.linear.x / 0.1
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f'Received parameters - Radius: {self.radius}, Angular Velocity: {self.angular_velocity}')

    def timer_callback(self):
        self.time_elapsed += self.timer_period

        x, y, theta = self.calculate_position()

        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta

        self.teleport_future = self.teleport_client.call_async(req)
        self.teleport_future.add_done_callback(self.teleport_callback)

    def calculate_position(self):
        a = self.radius  
        omega = self.angular_velocity 

        x = a * sin(omega * self.time_elapsed) + 5.5  
        y = a * sin(omega * self.time_elapsed) * cos(omega * self.time_elapsed) + 5.5 

        theta = 2 * omega * self.time_elapsed

        return x, y, theta

    def teleport_callback(self, future):
        try:
            future.result()
            self.get_logger().info('Teleportation successful')
        except Exception as e:
            self.get_logger().error(f'Teleportation failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = InfinityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
