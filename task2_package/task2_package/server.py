import rclpy
from rclpy.node import Node
from task2_interface.srv import Trajectory

class TrajectoryServer(Node):
    def __init__(self):
        super().__init__('trajectory_server')
        self.srv = self.create_service(Trajectory, 'compute_trajectory', self.compute_trajectory_callback)
        self.declare_parameter('initial_x', 5.5)
        self.declare_parameter('initial_y', 5.5)  
        self.declare_parameter('time_steps', 100)
        self.get_logger().info('Trajectory server is ready.')

    def compute_trajectory_callback(self, request, response):
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        time_steps = self.get_parameter('time_steps').value

        x, y = request.x, request.y
        vx, vy = request.vx, request.vy

        trajectory_x = []
        trajectory_y = []

        for t in range(time_steps):
            new_x = x + vx * t * 0.1
            new_y = y + vy * t * 0.1
            trajectory_x.append(new_x)
            trajectory_y.append(new_y)

        response.trajectory_x = trajectory_x
        response.trajectory_y = trajectory_y

        return response

def main(args=None):
    rclpy.init(args=args)
    trajectory_server = TrajectoryServer()

    rclpy.spin(trajectory_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
