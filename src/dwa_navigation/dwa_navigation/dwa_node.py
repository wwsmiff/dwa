import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        self.timer = self.create_timer(0.1, self.navigate)

        self.pose = None
        self.velocity = None
        self.scan = None
        self.goal = None

        self.get_logger().info("Custom DWA Planner Started")

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        self.velocity = msg.twist.twist

    def scan_cb(self, msg):
        self.scan = msg

    def goal_cb(self, msg):
        self.goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        self.get_logger().info(
            f"New goal received: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}"
        )
 
    def navigate(self):
        if self.pose is None or self.scan is None or self.goal is None:
            return
        x, y, _ = self.get_pose()

        if np.linalg.norm(np.array([x, y]) - self.goal) < 0.2:
            stop = Twist()
            self.cmd_pub.publish(stop)
            self.get_logger().info("Goal reached")
            return
        
        best_cost = float('inf')
        best_cmd = (0.0, 0.0)

        v_min, v_max, w_min, w_max = self.dynamic_window()
        v_range = np.linspace(max(0.05, v_min), v_max, 5)
        w_range = np.linspace(w_min, w_max, 7)


        for v in v_range:
            for w in w_range:
                traj = self.predict_trajectory(v, w)
                cost = self.evaluate_trajectory(traj, v, w)

                if cost < best_cost:
                    best_cost = cost
                    best_cmd = (v, w)

        cmd = Twist()
        cmd.linear.x = best_cmd[0]
        cmd.angular.z = best_cmd[1]
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(f"Selected v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}")

    def predict_trajectory(self, v, w):
        x, y, theta = self.get_pose()
        traj = []

        dt = 0.1
        for _ in range(20):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            traj.append((x, y))

        return traj

    def evaluate_trajectory(self, traj, v, w):
        obs_cost = self.obstacle_cost(traj)
        if obs_cost == float('inf'):
            return float('inf')
        goal_cost = np.linalg.norm(np.array(traj[-1]) - self.goal)
        obstacle_cost = self.obstacle_cost(traj)
        smoothness_cost = abs(traj[0][0] - traj[-1][0])
        heading_cost = self.heading_cost(traj, w)
        vel_cost = self.velocity_cost(v)

        return (1.0 * goal_cost + 
               1.5 * obstacle_cost +
               0.1 * smoothness_cost +
               0.5 * heading_cost +
               0.2 * vel_cost)

    def obstacle_cost(self, traj):
        min_dist = float('inf')

        rx, ry, rtheta = self.get_pose()
        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if r < self.scan.range_max:
                ox_r = r * math.cos(angle)
                oy_r = r * math.sin(angle)

                ox = rx + ox_r * math.cos(rtheta) - oy_r * math.sin(rtheta)
                oy = ry + ox_r * math.sin(rtheta) + oy_r * math.cos(rtheta)

                for tx, ty in traj:
                    dist = math.hypot(tx - ox, ty - oy)
                    min_dist = min(min_dist, dist)

            angle += self.scan.angle_increment

        if min_dist < 0.25:
            return float('inf') 

        return 1.0 / min_dist
        
    def get_pose(self):
        x = self.pose.position.x
        y = self.pose.position.y
        q = self.pose.orientation
        theta = math.atan2(2*(q.w*q.z + q.x*q.y),
                           1 - 2*(q.y*q.y + q.z*q.z))
        return x, y, theta
    
    def dynamic_window(self):
        v = self.velocity.linear.x
        w = self.velocity.angular.z

        v_min = max(0.0, v - 0.1)
        v_max = min(0.22, v + 0.1)

        w_min = w - 0.5
        w_max = w + 0.5

        return v_min, v_max, w_min, w_max

    def heading_cost(self, traj, w):
        _, _, theta = self.get_pose()
        predicted_theta = theta + w * 2.0

        goal_angle = math.atan2(
            self.goal[1] - self.get_pose()[1],
            self.goal[0] - self.get_pose()[0]
        )

        error = goal_angle - predicted_theta

        return abs(math.atan2(math.sin(error), math.cos(error)))

    def velocity_cost(self, v):
        return 1.0 / (v + 0.01)


def main():
    rclpy.init()
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
