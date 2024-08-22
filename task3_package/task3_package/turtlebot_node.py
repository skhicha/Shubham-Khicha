import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from task3_interface.srv import ComputeAngVel
from std_msgs.msg import Float64

class TurtleBotNode(Node):

    def __init__(self):
        super().__init__('turtlebot_node')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.radius_subscriber = self.create_subscription(
            Float64,
            '/radius',
            self.radius_callback,
            10
        )

        self.client = self.create_client(ComputeAngVel, 'compute_ang_vel')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Service available.')

    def radius_callback(self, msg):
        request = ComputeAngVel.Request()
        request.radius = msg.data

        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received angular velocity: {response.angular_velocity}')

            twist = Twist()
            twist.linear.x = 0.1  
            twist.angular.z = response.angular_velocity
            self.cmd_vel_publisher.publish(twist)
            
            self.get_logger().info(f'Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()