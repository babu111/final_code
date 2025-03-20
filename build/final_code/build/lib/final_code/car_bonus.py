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
        self.move(linear_speed=0.12, duration=0.1)

        # turn right 30 degree
        radian = 30 * 3.14 / 180
        self.move(angular_speed=-radian, duration=1.0)


        self.move(linear_speed=0.12, duration=11.2)

        # turn left 75 degree
        radian = 90 * 3.14 / 180
        self.move(angular_speed=radian, duration=1.0)

        self.move(linear_speed=0.12, duration=14)

        # for 94
        # radian = 95 * 3.14 / 180
        # for 92
        radian = 92 * 3.14 / 180
        self.move(angular_speed=-radian, duration=1.0)
        # for 94
        # self.move(linear_speed=0.12, duration=9.6)
        # for 92
        self.move(linear_speed=0.12, duration=9.2)        

        # turn right 150 degree
        radian = 165 * 3.14 / 180
        self.move(angular_speed=-radian/2, duration=2.0)

        # takes around 47 seconds to complete the first round

        time.sleep(60)
        # time.sleep(5)

        self.move(linear_speed=0.12, duration=0.2)

        # turn right 25 degree
        radian = 25 * 3.14 / 180
        self.move(angular_speed=-radian, duration=1.0)
        
        self.move(linear_speed=0.12, duration=9.5)

        # turn left 80 degree
        radian = 85 * 3.14 / 180
        self.move(angular_speed=radian, duration=1.0)
        
        self.move(linear_speed=0.12, duration=15)

        # turn right 92 degree
        radian = 86 * 3.14 / 180
        self.move(angular_speed=-radian, duration=1.0)
        
        self.move(linear_speed=0.12, duration=9.7)

        radian = 165 * 3.14 / 180
        self.move(angular_speed=-radian/2, duration=2.0)
        
        self.get_logger().info("Motion sequence completed.")


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    controller.execute_sequence()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
