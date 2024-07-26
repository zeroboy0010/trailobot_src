import rclpy
import sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, TransformStamped, Twist, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
from rclpy.time import Time
from time import time,sleep

import math
import numpy as np
import casadi as ca
from casadi import sin, cos, pi, arctan2

## complimentary filter 
delta = 0.90

wheel_radius = 0.1 # in meters
D = 0.5                    # in meters
DT = 1/20
x_init = 0.0
y_init = 0.0
theta_init = 0.0


# def pi_2_pi(angle):
#     while(angle > math.pi):
#         angle = angle - 2.0 * math.pi

#     while(angle < -math.pi):
#         angle = angle + 2.0 * math.pi

#     return angle

def euler_from_quaternion(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z +x * y)
    t4 = 1.0 - 2.0*(y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return yaw

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q



class odometry(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.feedback_sub = self.create_subscription(Vector3, '/position_uros', self.feedback_callback, 20)
        self.odometry_pub = self.create_publisher(Odometry, '/odom', 10)
        self.state_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odometry_timer = self.create_timer(DT , self.odometry_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.position = [0.0, 0.0 , 0.0]
        self.u_to_amcl = [0.0,0.0]
        self.pose_x = 0.0
        self.pose_y = 0.0

        self.assigh = 0

    def cmd_callback(self, vel_msg):
        # vel_msg = Twist()
        self.u_to_amcl[0] = vel_msg.linear.x
        self.u_to_amcl[1] = vel_msg.angular.z

    def odometry_callback(self):
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_link"

        # Velocity
        odometry_msg.twist.twist.linear.x = self.u_to_amcl[0]
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.angular.z = self.u_to_amcl[1]

        # Position 
        odometry_msg.pose.pose.position.x = float(self.position[0])
        odometry_msg.pose.pose.position.y = float(self.position[1])
        odometry_msg.pose.pose.position.z = 0.0
        self.q = quaternion_from_euler(0, 0, self.position[2])

        print(self.position)
        odometry_msg.pose.pose.orientation.x = self.q[0]
        odometry_msg.pose.pose.orientation.y = self.q[1]
        odometry_msg.pose.pose.orientation.z = self.q[2]
        odometry_msg.pose.pose.orientation.w = self.q[3]
        self.odometry_pub.publish(odometry_msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id =  'base_link'
        t.transform.translation.x = self.pose_x + float(self.position[0])
        t.transform.translation.y = self.pose_y + float(self.position[1])
        t.transform.translation.z = 0.0
        # quat = quaternion_from_euler(0.0, 0.0, self.xEst[2])
        t.transform.rotation.x = self.q[0]
        t.transform.rotation.y = self.q[1]
        t.transform.rotation.z = self.q[2]
        t.transform.rotation.w = self.q[3]
        self.tf_broadcaster.sendTransform(t)
        ########################
        
    def feedback_callback(self, vel_msg):
        self.position = [vel_msg.x, vel_msg.y , vel_msg.z]

    
    def pi_2_pi(self, angle):
        return self.angle_mod(angle)


    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        if isinstance(x, float):
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle
        



def main(args=None):
    rclpy.init(args=args)
    odometry_node = odometry()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()