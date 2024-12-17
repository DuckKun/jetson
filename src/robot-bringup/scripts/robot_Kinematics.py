#!/usr/bin/env python3

import sys
sys.path.append('/home/codyjetson/ros2Bot/src/robot-bringup/scripts')
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import jetson_robot_control
import numpy as np


ROBOT_WHEEL_RADIUS = 0.0725
ROBOT_MOTOR_PPR = 500
ROBOT_WHEEL_SEPARATION = 0.315
ROBOT_MAX_LINEAR_M_S = 0.063
ROBOT_MIN_LINEAR_M_S = -0.063
ROBOT_MAX_ANGULAR_R_S_p = 0.25
ROBOT_MIN_ANGULAR_R_S_p = 0.15
ROBOT_MAX_ANGULAR_R_S_n = -0.15
ROBOT_MIN_ANGULAR_R_S_n = -0.25
ROBOT_MAX_ANGULAR_R_S = 0.2
ROBOT_MIN_ANGULAR_R_S = -0.1

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

@dataclass
class robotOdom: 
    x_pos: float 
    y_pos: float
    theta: float

class _RobotOdometry(Node):
    def __init__(self):
        super().__init__("robot_odom_cal_node")
        self.twist_subscription = self.create_subscription(
            Twist, "cmd_vel", self.twist_callback, 10
        )
        self.robot = jetson_robot_control.RobotControlNode()
        self.twist_subscription
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.twist = Twist()
        self.pub_period = 0.01 # 0.024
        self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x_pos = 0
        self.y_pos = 0
        self.theta = 0
        self.onceFlag = False
        self.turnRightFlag = False
        self.turnLeftFlag = False


    def pub_callback(self):
        robot_state = self.robot.read_data()

        if robot_state is None:
            return
        else:
            self._calculateOdom(robot_state._vl, robot_state._vr)

        robot_orientation = quaternion_from_euler(0, 0, self.theta)
        timestamp = self.get_clock().now().to_msg()
        # transforms
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x_pos
        t.transform.translation.y = self.y_pos
        t.transform.translation.z = self.theta
        t.transform.rotation = robot_orientation

        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.header.stamp = timestamp
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = self.theta
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = robot_state.v
        odom_msg.twist.twist.angular.z = robot_state.w

        # broadcast and publish
        self.tf_broadcaster.sendTransform(t)
        self.odom_publisher.publish(odom_msg)

    def twist_callback(self, twist: Twist):
        self.twist = twist
        self._setUnicycle(twist.linear.x, twist.angular.z)
        


    def _calculateOdom(self, 
                        # dl_ticks: int, dr_ticks: int
                        _vl: float, _vr: float):
        # delta_l = (2 * math.pi * ROBOT_WHEEL_RADIUS * dl_ticks) / ROBOT_MOTOR_PPR
        # delta_r = (2 * math.pi * ROBOT_WHEEL_RADIUS * dr_ticks) / ROBOT_MOTOR_PPR
        # delta_center = (delta_l + delta_r) / 2

        if self.twist.linear.x == 0 and self.twist.angular.z == 0:
            _vl = 0
            _vr = 0

        delta_l = _vl * 0.23 * ROBOT_WHEEL_RADIUS #*
        delta_r = _vr * 0.23 * ROBOT_WHEEL_RADIUS #*
        delta_center = (delta_l + delta_r) / 2

        self.x_pos += (float)(delta_center * math.cos(self.theta))
        self.y_pos += (float)(delta_center * math.sin(self.theta))
        self.theta += (float)((delta_l - delta_r)/ROBOT_WHEEL_SEPARATION)

        self.get_logger().info(f"x_pos: {self.x_pos}, y_pos: {self.y_pos}, theta: {self.theta} ")

        # return self.x_pos, self.y_pos, self.theta

    def _setUnicycle(self, v: float, w: float):
        if(v > ROBOT_MAX_LINEAR_M_S): v = ROBOT_MAX_LINEAR_M_S
        if(v < ROBOT_MIN_LINEAR_M_S): v = ROBOT_MIN_LINEAR_M_S

        if (np.abs(v) - 0.02 <= 0 and w != 0) : #rotate in place
            if (self.onceFlag == False):
                if (w <= -0.01): self.turnRightFlag = True
                if (w >= 0.01): self.turnLeftFlag = True
                self.onceFlag = True

            if (self.turnRightFlag  == 1): w = -0.2
            if (self.turnRightFlag  == 1): w = 0.2
        elif (np.abs(v) - 0.02 >= 0 and w != 0):
            if (abs(w) < 0.12): w = 0 
            if(w > ROBOT_MAX_ANGULAR_R_S): w = ROBOT_MAX_ANGULAR_R_S
            if(w < ROBOT_MIN_ANGULAR_R_S): w = ROBOT_MIN_ANGULAR_R_S

        if (v == 0 and w == 0):
            self.onceFlag = False
            self.turnLeftFlag  = False
            self.turnRightFlag  = False
            v  = 0
            w = 0
        self.v_l = (2 * v - w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS)
        self.v_r = (2 * v + w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS)
        self.robot.send_command(self.v_l, self.v_r)



def main(args=None):
    rclpy.init(args=args)
    robot_odom_node = _RobotOdometry()
    while rclpy.ok():
        rclpy.spin_once(robot_odom_node)

    robot_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()