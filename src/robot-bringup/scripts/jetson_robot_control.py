#!/usr/bin/env python3

from dataclasses import dataclass
import math
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from tf2_ros import TransformBroadcaster

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry


# def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
#     cy = math.cos(yaw * 0.5)
#     sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5)
#     sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5)
#     sr = math.sin(roll * 0.5)

#     q = Quaternion()
#     q.w = cy * cp * cr + sy * sp * sr
#     q.x = cy * cp * sr - sy * sp * cr
#     q.y = sy * cp * sr + cy * sp * cr
#     q.z = sy * cp * cr - cy * sp * sr

#     return q


@dataclass
class SerialStatus:
    """Class for different data given by the embedded system"""

    # left_ref_speed: float
    # right_ref_speed: float
    # left_speed:float
    # right_speed: float
    # x_pos: float
    # y_pos: float
    # theta: float
    # left_effort: float
    # right_effor: float
    v: float
    w: float
    _vl: float
    _vr: float 



class RobotControlNode(Node):
    def __init__(self):
        super().__init__("jetson_robot_node")
        self.declare_parameter("jetson_port", value="/dev/ttyACM0")
        # self.twist_subscription = self.create_subscription(
        #     Twist, "cmd_vel", self.twist_callback, 10
        # )
        # self.twist_subscription

        # self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        # time.sleep(0.2)
        self.port = self.get_parameter("jetson_port").get_parameter_value().string_value
        self.ser = serial.Serial(
            self.port,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            rtscts=False,
        )
        if self.ser:
            self.get_logger().info(f"Connected")
        self.get_logger().info(f"Using serial port {self.ser.name}")
        self.ser.flushInput()
        self.ser.flushOutput()
        # self.twist = Twist()
        # self.pub_period = 0.03  # 0.024
        # self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)
        # tf
        # self.tf_broadcaster = TransformBroadcaster(self)

    # def pub_callback(self):
    #     self.send_command(self.twist.linear.x, self.twist.angular.z)

    #     robot_state = self.read_data()
    #     if robot_state is None:
    #         return

    #     robot_orientation = quaternion_from_euler(0, 0, robot_state.theta)
    #     timestamp = self.get_clock().now().to_msg()
    #     # transforms
    #     t = TransformStamped()
    #     t.header.stamp = timestamp
    #     t.header.frame_id = "odom"
    #     t.child_frame_id = "base_link"
    #     t.transform.translation.x = robot_state.x_pos
    #     t.transform.translation.y = robot_state.y_pos
    #     t.transform.translation.z = robot_state.theta
    #     t.transform.rotation = robot_orientation

    #     # odometry twist
    #     odom_msg = Odometry()
    #     odom_msg.header.frame_id = "odom"
    #     odom_msg.child_frame_id = "base_link"
    #     odom_msg.header.stamp = timestamp
    #     odom_msg.pose.pose.position.x = robot_state.x_pos
    #     odom_msg.pose.pose.position.y = robot_state.y_pos
    #     odom_msg.pose.pose.position.z = robot_state.theta
    #     odom_msg.pose.pose.orientation = robot_orientation
    #     odom_msg.twist.twist.linear.x = robot_state.v
    #     odom_msg.twist.twist.angular.z = robot_state.w

    #     # broadcast and publish
    #     self.tf_broadcaster.sendTransform(t)
    #     self.odom_publisher.publish(odom_msg)

    def send_command(self, 
                    #  linear: float, angular: float
                    vl: float, vr: float
                     ):
        self.get_logger().debug(f"Data to send: {vl}, {vr}")
        command = f"{vl:.3f}/{vr:.3f}".encode("UTF-8")
        self.get_logger().debug(f'Sending command: "{command}"')
        self.ser.write(command)

    def read_data(self) -> SerialStatus:
        # time.sleep(0.3)
        while self.ser.in_waiting == 0:
            pass

        res = self.ser.readline()  # self.ser.in_waiting
        res = res.strip().split(b"/")
        # self.get_logger().info(f'data: "{res}", byte: {len(res)}')
        # print(res)
        try:
            values_list = [float(value) for value in res]
        except ValueError as e:
            self.get_logger().warn(f'Bad data: "{res}"')
            return None

        return SerialStatus(*values_list)

    # def twist_callback(self, twist: Twist):
    #     self.twist = twist


# def main(args=None):
#     rclpy.init(args=args)
#     robot_control_node = RobotControlNode()
#     while rclpy.ok():
#         rclpy.spin_once(robot_control_node)

#     robot_control_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
