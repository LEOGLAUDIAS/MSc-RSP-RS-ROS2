import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        # Publish to teleop_cmd instead of cmd_vel
        self.publisher = self.create_publisher(Twist, 'teleop_cmd', 10)
        self.get_logger().info("Teleop Ready! Use WASD keys to move, Q to quit.")

    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':
                twist.linear.x = 0.2
            elif key == 's':
                twist.linear.x = -0.2
            elif key == 'a':
                twist.angular.z = 0.5
            elif key == 'd':
                twist.angular.z = -0.5
            elif key == 'q':
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
