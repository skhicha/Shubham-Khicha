import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist
from math import sin, cos, pi

class InfinityPublisher(Node):
    def __init__(self):
        super().__init__('infinity_publisher')

        # Client for the TeleportAbsolute service
        self.teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport_absolute service...')

        # Subscription to /cmd_vel topic from turtlebot_mover node
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for controlling the teleportation
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize variables
        self.time_elapsed = 0.0
        self.radius = 1.0
        self.angular_velocity = 0.0

    def cmd_vel_callback(self, msg):
        # Extract radius from linear.x and angular velocity from angular.z
        self.radius = msg.linear.x / 0.1
        self.angular_velocity = msg.angular.z
        self.get_logger().info(f'Received parameters - Radius: {self.radius}, Angular Velocity: {self.angular_velocity}')

    def timer_callback(self):
        self.time_elapsed += self.timer_period

        # Calculate the position and orientation for the circular infinity shape
        x, y, theta = self.calculate_position()

        # Prepare and send the teleport request
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta

        # Call the teleport_absolute service
        self.teleport_future = self.teleport_client.call_async(req)
        self.teleport_future.add_done_callback(self.teleport_callback)

    def calculate_position(self):
        # Parameters for the circular infinity shape
        a = self.radius  # Scaling factor
        omega = self.angular_velocity  # Use the computed angular velocity

        # Calculate the circular figure-eight pattern
        # x oscillates horizontally and y oscillates vertically in a circular pattern
        x = a * sin(omega * self.time_elapsed) + 5.5  # Horizontal movement
        y = a * sin(omega * self.time_elapsed) * cos(omega * self.time_elapsed) + 5.5  # Vertical movement

        # Calculate orientation (theta) for the circular infinity shape
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
