import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        # Publishers & subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.teleop_sub = self.create_subscription(Twist, 'teleop_cmd', self.teleop_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.state_sub = self.create_subscription(String, 'fsm_state', self.state_callback, 10)

        # FSM states
        self.states = ["IDLE", "TELEOP", "AUTONOMOUS", "AVOIDING", "GOAL_REACHED"]
        self.current_state = "IDLE"
        self.get_logger().info(f"FSM started in state: {self.current_state}")

        # Data storage
        self.teleop_cmd = Twist()
        self.obstacle_near = False

        # Timer to run FSM loop
        self.timer = self.create_timer(0.5, self.state_machine)

    def teleop_callback(self, msg: Twist):
        self.teleop_cmd = msg

    def scan_callback(self, msg: LaserScan):
        front = min(msg.ranges[0:30] + msg.ranges[-30:])
        self.obstacle_near = front < 0.6

    def state_callback(self, msg: String):
        if msg.data in self.states:
            self.get_logger().info(f"Switching state to: {msg.data}")
            self.current_state = msg.data
        else:
            self.get_logger().warn(f"Unknown state command: {msg.data}")

    def state_machine(self):
        twist = Twist()

        if self.current_state == "IDLE":
            self.get_logger().info("State: IDLE")
            twist = Twist()  # stop

        elif self.current_state == "TELEOP":
            self.get_logger().info("State: TELEOP")
            twist = self.teleop_cmd

        elif self.current_state == "AUTONOMOUS":
            self.get_logger().info("State: AUTONOMOUS")
            if self.obstacle_near:
                self.current_state = "AVOIDING"
            else:
                twist.linear.x = 0.2

        elif self.current_state == "AVOIDING":
            self.get_logger().info("State: AVOIDING")
            twist.angular.z = 0.5
            if not self.obstacle_near:
                self.current_state = "AUTONOMOUS"

        elif self.current_state == "GOAL_REACHED":
            self.get_logger().info("State: GOAL_REACHED")
            twist = Twist()  # stop

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
