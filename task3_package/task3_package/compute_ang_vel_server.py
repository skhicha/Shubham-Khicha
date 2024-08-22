import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from task3_interface.srv import ComputeAngVel

class ComputeAngularVelocityServer(Node):
    def __init__(self):
        super().__init__('compute_ang_vel_server')
        self.srv = self.create_service(ComputeAngVel, 'compute_ang_vel', self.compute_ang_vel_callback)
        self.radius_subscription = self.create_subscription(
            Float64,
            '/radius',
            self.radius_callback,
            10
        )
        self.linear_velocity = 0.1 
        self.radius = None

    def radius_callback(self, msg):
        self.radius = msg.data
        self.get_logger().info(f'Received radius: {self.radius}')

    def compute_ang_vel_callback(self, request, response):
        if self.radius is None or self.radius <= 0:
            self.get_logger().info('Invalid or no radius received.')
            response.angular_velocity = 0.0
        else:
            angular_velocity = self.linear_velocity / self.radius
            response.angular_velocity = angular_velocity
            self.get_logger().info(f'Radius: {self.radius} | Angular Velocity: {angular_velocity:.3f} rad/s')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ComputeAngularVelocityServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()