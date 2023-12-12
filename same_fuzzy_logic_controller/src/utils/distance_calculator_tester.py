import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DistanceCalculator:
    def __init__(self):
        self.distance = 0.0
        self.last_odom = None

        self.node = rclpy.create_node('distance_calculator')
        self.sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        if self.last_odom is not None:
            # Calculate the distance between the last position and the current position
            delta_x = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            distance = math.sqrt(delta_x**2 + delta_y**2)

            # Add the distance to the total distance traveled
            self.distance += distance
           # print(distance)

        # Save the current odom message for the next iteration
        self.last_odom = msg

        print(f"Distance traveled: {self.distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    distance_calculator = DistanceCalculator()
    rclpy.spin(distance_calculator.node)
    distance_calculator.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()