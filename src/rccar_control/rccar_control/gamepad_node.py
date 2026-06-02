#!/usr/bin/env python3
"""
Gamepad Control Node
Supports Xbox (wired) and Game Pad (wireless) controllers
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math


class MecanumController(Node):
    """Convert gamepad input to mecanum wheel velocities"""
    
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # Robot dimensions (from URDF)
        self.wb = 0.400  # wheel base (front-rear)
        self.tw = 0.340  # track width (left-right)
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 2.0  # rad/s
        
        # Gamepad mappings (Xbox/generic layout)
        # Axes: 0=LX, 1=LY, 2=LT, 3=RX, 4=RY, 5=RT
        # Buttons: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=back, 7=start
        
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.wheel_vel_pub = self.create_publisher(
            Twist, 'wheel_velocities', 10)  # For debugging
        
        self.get_logger().info('Mecanum controller initialized')
    
    def joy_callback(self, msg: Joy):
        """
        Convert joystick input to robot velocity
        Xbox/Gamepad layout:
        - Left stick: linear motion (forward/back, strafe left/right)
        - Right stick: rotation
        - LT/RT: fine control (scale down)
        """
        # Deadzone
        deadzone = 0.1
        
        # Left stick (linear motion)
        lx = msg.axes[0] if abs(msg.axes[0]) > deadzone else 0.0  # strafe
        ly = -msg.axes[1] if abs(msg.axes[1]) > deadzone else 0.0  # forward/back
        
        # Right stick (rotation)
        rx = msg.axes[3] if abs(msg.axes[3]) > deadzone else 0.0  # rotation
        
        # Triggers for fine control (0~1)
        lt = max(0, msg.axes[2])  # Left trigger (0 to 1)
        rt = max(0, msg.axes[5])  # Right trigger (0 to 1)
        
        # Combine triggers: if either is pressed, reduce velocity
        fine_control = (lt + rt) / 2.0
        speed_scale = 1.0 - (fine_control * 0.5)  # 50% reduction at full trigger
        
        # Calculate linear and angular velocity
        vx = ly * self.max_linear_vel * speed_scale      # forward/back
        vy = lx * self.max_linear_vel * speed_scale      # strafe left/right
        wz = rx * self.max_angular_vel * speed_scale     # rotation
        
        # Publish cmd_vel (for rviz and testing)
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = wz
        self.cmd_vel_pub.publish(twist)
        
        # Calculate individual wheel velocities for mecanum drive
        # Mecanum kinematics:
        # v_fl = (vx - vy - (wb+tw)/2 * wz) / wheel_r
        # v_fr = (vx + vy + (wb+tw)/2 * wz) / wheel_r
        # v_rl = (vx + vy - (wb+tw)/2 * wz) / wheel_r
        # v_rr = (vx - vy + (wb+tw)/2 * wz) / wheel_r
        
        L = (self.wb + self.tw) / 2.0
        
        v_fl = vx - vy - L * wz
        v_fr = vx + vy + L * wz
        v_rl = vx + vy - L * wz
        v_rr = vx - vy + L * wz
        
        # Publish wheel velocities for debugging
        wheel_vel = Twist()
        wheel_vel.linear.x = v_fl
        wheel_vel.linear.y = v_fr
        wheel_vel.linear.z = v_rl
        wheel_vel.angular.x = v_rr
        self.wheel_vel_pub.publish(wheel_vel)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
