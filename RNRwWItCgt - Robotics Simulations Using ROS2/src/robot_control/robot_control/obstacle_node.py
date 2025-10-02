import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        self.subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def laser_callback(self, msg: LaserScan):
        twist = Twist()

        # Divide into regions
        ranges = list(msg.ranges)
        front = min(ranges[0:30] + ranges[-30:])       # ±30°
        left = min(ranges[60:120])                     # left sector
        right = min(ranges[-120:-60])                  # right sector

        threshold = 0.6  # meters

        if front < threshold:
            # obstacle ahead → turn toward freer side
            if left > right:
                twist.angular.z = 0.5
            else:
                twist.angular.z = -0.5
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
