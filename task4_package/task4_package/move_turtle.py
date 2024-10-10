import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class MoveTurtleNode(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Publishers for turtles' cmd_vel topics
        self.publisher1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.publisher3 = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)

        # Set velocities and radii for circles
        self.radius_upper = 1.5  # Radius for the upper circle
        self.radius_lower = 1.5 
        self.linear_velocity_1 = 1.0 # Speed for Turtle 1
        self.angular_velocity_1 = - self.linear_velocity_1 / self.radius_upper
        self.linear_velocity_2 = 1.0  # Speed for Turtle 2
        self.angular_velocity_2 = self.linear_velocity_2 / self.radius_upper
        self.linear_velocity_3 = 0.5  # Speed for Turtle 3 
        self.angular_velocity_3 = self.linear_velocity_3 / self.radius_lower

        # Pose variables for each turtle
        self.pose1 = None
        self.pose2 = None
        self.pose3 = None

        # Subscribe to pose updates
        self.create_subscription(Pose, '/turtle1/pose', self.update_pose1, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.update_pose2, 10)
        self.create_subscription(Pose, '/turtle3/pose', self.update_pose3, 10)

        # Spawn the turtles
        self.spawn_turtles()

        # Timer to control movement
        self.create_timer(0.1, self.move_infinity)

    def update_pose1(self, msg):
        self.pose1 = msg

    def update_pose2(self, msg):
        self.pose2 = msg

    def update_pose3(self, msg):
        self.pose3 = msg

    def spawn_turtles(self):
        """Spawn three turtles and delete the default turtle."""
        # Kill the default turtle1
        kill_client = self.create_client(Kill, 'kill')
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        kill_client.call_async(kill_request)
        self.get_logger().info('Default turtle1 killed')

        # Spawn client
        spawn_client = self.create_client(Spawn, 'spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # Spawn Turtle 1 at (5.0, 8.0) for the upper circle (anticlockwise)
        spawn_request_1 = Spawn.Request()
        spawn_request_1.x = 3.5
        spawn_request_1.y = 5.5
        spawn_request_1.theta = math.pi / 2  # Start with negative theta (anticlockwise)
        spawn_request_1.name = 'turtle1'
        spawn_client.call_async(spawn_request_1)
        self.get_logger().info('Turtle 1 spawned at ()')

        # Spawn Turtle 2 at (5.0, 6.0) for the upper circle (clockwise)
        spawn_request_2 = Spawn.Request()
        spawn_request_2.x = 6.5
        spawn_request_2.y = 5.5  # Same y-position (upper circle)
        spawn_request_2.theta = math.pi / 2  # Start with positive theta (clockwise)
        spawn_request_2.name = 'turtle2'
        spawn_client.call_async(spawn_request_2)
        self.get_logger().info('Turtle 2 spawned at (8.0, 8.0)')

        # Spawn Turtle 3 at (5.0, 4.0) for the lower circle (clockwise)
        spawn_request_3 = Spawn.Request()
        spawn_request_3.x = 5.0
        spawn_request_3.y = 4.0  # Lower circle
        spawn_request_3.theta = math.pi  # Start with positive theta (clockwise)
        spawn_request_3.name = 'turtle3'
        spawn_client.call_async(spawn_request_3)
        self.get_logger().info('Turtle 3 spawned at (5.0, 4.0)')

    def move_infinity(self):
        twist1 = Twist()
        twist2 = Twist()
        twist3 = Twist()

        # Detect angular proximity and reverse direction on collision
        if self.pose1 and self.pose2 and self.pose3:
            if self.are_about_to_collide(self.pose1, self.pose2):
                # Reverse direction for both Turtle 1 and Turtle 2
                self.linear_velocity_1 *= -1
                self.angular_velocity_1 = - self.linear_velocity_1 / self.radius_upper
                self.linear_velocity_2 *= -1
                self.angular_velocity_2 = self.linear_velocity_2 / self.radius_upper

            if self.are_about_to_collide(self.pose1, self.pose3):
                # Reverse direction for both Turtle 1 and Turtle 3
                self.linear_velocity_1 *= -1
                self.angular_velocity_1 = - self.linear_velocity_1 / self.radius_upper
                self.linear_velocity_3 *= -1
                self.angular_velocity_3 = self.linear_velocity_3 / self.radius_lower

            if self.are_about_to_collide(self.pose2, self.pose3):
                # Reverse direction for both Turtle 2 and Turtle 3
                self.linear_velocity_2 *= -1
                self.angular_velocity_2 = self.linear_velocity_2 / self.radius_upper
                self.linear_velocity_3 *= -1
                self.angular_velocity_3 = self.linear_velocity_3 / self.radius_lower

        # Movement for Turtle 1 (Upper Circle, anticlockwise or clockwise)
        twist1.linear.x = self.linear_velocity_1
        twist1.angular.z = self.angular_velocity_1

        # Movement for Turtle 2 (Upper Circle, clockwise or anticlockwise)
        twist2.linear.x = self.linear_velocity_2
        twist2.angular.z = self.angular_velocity_2

        # Movement for Turtle 3 (Lower Circle, clockwise or anticlockwise)
        twist3.linear.x = self.linear_velocity_3
        twist3.angular.z = self.angular_velocity_3

        # Publish movements
        self.publisher1.publish(twist1)
        self.publisher2.publish(twist2)
        self.publisher3.publish(twist3)

    def are_about_to_collide(self, pose_a, pose_b, threshold=0.5):
        """Check if two turtles are about to collide based on their positions."""
        distance = math.sqrt((pose_a.x - pose_b.x) ** 2 + (pose_a.y - pose_b.y) ** 2)
        return distance < threshold

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
