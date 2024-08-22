import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from task3_interface.srv import ComputeAngVel
from std_msgs.msg import Float64

class TurtleBotNode(Node):

    def __init__(self):
        super().__init__('turtlebot_node')

        # Create a publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a subscriber for the /radius topic
        self.radius_subscriber = self.create_subscription(
            Float64,
            '/radius',
            self.radius_callback,
            10
        )

        # Create a client for the compute_ang_vel service
        self.client = self.create_client(ComputeAngVel, 'compute_ang_vel')
        
        # Check if the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Service available.')

    def radius_callback(self, msg):
        # Create a request for the compute_ang_vel service
        request = ComputeAngVel.Request()
        request.radius = msg.data

        # Send the request and wait for the result
        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received angular velocity: {response.angular_velocity}')

            # Publish the Twist message
            twist = Twist()
            twist.linear.x = 0.1  # Constant linear velocity
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