import rclpy
from rclpy.node import Node
from task2_interface.srv import Trajectory
from turtlesim.srv import TeleportAbsolute, Spawn
import matplotlib.pyplot as plt
import time

class TrajectoryClient(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        self.cli = self.create_client(Trajectory, 'compute_trajectory')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Trajectory.Request()

        # Create clients for turtlesim services
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')
        self.teleport_req = TeleportAbsolute.Request()

        self.spawn_cli = self.create_client(Spawn, '/spawn')
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting again...')
        self.spawn_req = Spawn.Request()

    def send_request(self, x, y, vx, vy):
        self.req.x = x
        self.req.y = y
        self.req.vx = vx
        self.req.vy = vy
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def spawn_turtle(self, x, y, theta=0.0):
        self.spawn_req.x = x
        self.spawn_req.y = y
        self.spawn_req.theta = theta
        self.spawn_cli.call_async(self.spawn_req)

    def move_turtle(self, turtle_name, x, y):
        teleport_cli = self.create_client(TeleportAbsolute, f'/{turtle_name}/teleport_absolute')
        while not teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Teleport service for {turtle_name} not available, waiting again...')
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = x
        teleport_req.y = y
        teleport_req.theta = 0.0
        teleport_cli.call_async(teleport_req)

def plot_trajectory(trajectory_x, trajectory_y):
    plt.plot(trajectory_x, trajectory_y, marker='o')
    plt.title('Robot Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    trajectory_client = TrajectoryClient()

    x = float(input("Enter initial x: "))
    y = float(input("Enter initial y: "))
    vx = float(input("Enter velocity x: "))
    vy = float(input("Enter velocity y: "))

    response = trajectory_client.send_request(x, y, vx, vy)
    
    trajectory_x = response.trajectory_x
    trajectory_y = response.trajectory_y

    # Spawn a new turtle
    spawn_x = trajectory_x[0]
    spawn_y = trajectory_y[0]
    trajectory_client.spawn_turtle(spawn_x, spawn_y)

    # Move the new turtle along the trajectory
    turtle_name = 'turtle2'  # The name of the new turtle
    for i in range(len(trajectory_x)):
        trajectory_client.move_turtle(turtle_name, trajectory_x[i], trajectory_y[i])
        time.sleep(0.1)

    plot_trajectory(trajectory_x, trajectory_y)

    rclpy.shutdown()

if __name__ == '__main__':
    main()