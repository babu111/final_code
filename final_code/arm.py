#!/usr/bin/env python3
import time
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np
from rclpy.node import Node

class OdomCloseLoop(Node):
    def __init__(self, direction=1, target_distance=1.1):
        super().__init__('close_loop_odom')
        # debug mode
        self.target_distance = target_distance
        self.debug = True
        self.direction = direction
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # TODO: subscribe to /odom topic
        # Publisher to control the velocity
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # TODO: add the publisher to send velocity values
        self.position_x_list = []
        self.motion_move = Twist()
        self.motion_stop = Twist()
        self.movement_complete = False
        # Set constant speed to move forward
        self.motion_move.linear.x = 0.1 * direction
        # Set speed to stop
        self.motion_stop.linear.x = 0.0

    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        self.position_x_list.append(position_x)
        # Ensure the list has at least 2 elements
        if len(self.position_x_list) > 1:
            initial_position = self.position_x_list[0]
            traveled_distance = abs(position_x - initial_position)
            # Check if the last recorded position is greater than the first one
            # plus the desired distance to travel
            print(initial_position)
            print(position_x)
            if traveled_distance>= self.target_distance:
            # TODO: fill in the condition to check whether the last item on the list
            # is greater than the first one plus the desired distance to be traveled
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Reached goal, stopping...")
                self.movement_complete = True
            else:
                self.pub.publish(self.motion_move)
                if self.debug:
                    self.get_logger().info(f"Traveled distance: {traveled_distance}")


def pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2, put_pose3):
    arm.inverse_kinematic_movement(pre_pick_pose)
    time.sleep(0.5)
    gripper.move_to_position(0.0)
    arm.inverse_kinematic_movement(pick_pose)
    gripper.move_to_position(0.8)
    print("got the cube")
    time.sleep(0.5)
    arm.inverse_kinematic_movement(pre_put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(pre_put_pose2)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose2)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose3)
    time.sleep(0.5)
    gripper.move_to_position(0.0)
    print("finished!")

def set_pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
    return_pose = Pose()
    return_pose.position.x = position_x
    return_pose.position.y = position_y
    return_pose.position.z = position_z
    return_pose.orientation.x = orientation_x
    return_pose.orientation.y = orientation_y
    return_pose.orientation.z = orientation_z
    return_pose.orientation.w = orientation_w
    return return_pose

def main():
    rclpy.init()
    # odom_cl = OdomCloseLoop(target_distance=3.4)
    # while rclpy.ok() and not odom_cl.movement_complete:
    #     rclpy.spin_once(odom_cl)
    # odom_cl.destroy_node()
    # print("destroyed node")
    # time.sleep(3)

    # Initialize arm and gripper interfaces
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    arm.go_vertical()
    pre_pick_pose = set_pose(0.370, -0.092, 0.371, 0.817, 0.574, -0.036, 0.028)
    # bonus_pre_pick_pose = set_pose(0.380, -0.030, 0.371, 0.817, 0.574, -0.036, 0.028)
    pick_pose = set_pose(0.370, -0.092, 0.130, 0.758, 0.652, 0.008, 0.017)

    # bonus_pick_pose = set_pose(0.394, -0.031, 0.161, 0.751, 0.659, -0.025, 0.017)
    pre_put_pose1 = set_pose(-.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
    pre_put_pose2 = set_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
    put_pose1 = set_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
    put_pose2 = set_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)
    put_pose3 = set_pose(-0.159, 0.085, -0.336, -0.018, 0.997, 0.026, -0.076)

    # standard arm movement takes 2min 5s
    

    pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2, put_pose3)

    # odom_cl = OdomCloseLoop(direction=-1, target_distance = 3.4)
    # while rclpy.ok() and not odom_cl.movement_complete:
    #     rclpy.spin_once(odom_cl)
    # odom_cl.destroy_node()
    # print("destroyed node")

    # Shutdown ROS2 and clean up
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()

if __name__ == '__main__':
    main()



