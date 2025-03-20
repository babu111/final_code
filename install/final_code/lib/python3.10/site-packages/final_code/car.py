import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Allow publisher to initialize

    def move(self, linear_speed=0.0, angular_speed=0.0, duration=0.0):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        
        self.publisher_.publish(twist)
        time.sleep(duration)
        
        # Stop the robot after motion
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)

    def execute_sequence(self):
        # Move straight for 9.6s
        self.move(linear_speed=0.12, duration=3.19 / 0.12)
        

        # car runs 30s, arm moves 125s, so sleep 100s

        time.sleep(120)

        self.move(linear_speed=-0.12, duration=3.19 / 0.12)
        
        # Turn right 20 degrees (approximately)
        # self.move(angular_speed=-0.349, duration=1.0)  # -0.349 rad/s ≈ -20 deg in 1s
        
        # # Move straight for 9.6s
        # self.move(linear_speed=0.12, duration=9.6)
        
        # # Stop
        # self.move(duration=1.0)
        
        # # Move backward for 9.6s
        # self.move(linear_speed=-0.12, duration=9.6)
        
        # # Move forward 1.4 meters
        # self.move(linear_speed=0.12, duration=1.4 / 0.12)
        
        self.get_logger().info("Motion sequence completed.")


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    controller.execute_sequence()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
