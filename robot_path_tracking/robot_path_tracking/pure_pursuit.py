
import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

sampling_time = 0.1

# Parameters
k = 0.03  # look forward gain
Lfc = 0.3  # [m] look-ahead distance
Kp = 1  # speed proportional gain
Kp_pos = 1
WB = 0.3  # [m] wheel base of vehicle

# Parameters
N = 10  # Number of periods for the EMA
alpha = 2 / (N + 1)

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)




def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index

            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                if ind < len(self.cx) - 1 :
                    distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                            self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind


    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
        alpha = 0


    # print(alpha)

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def euler_from_quaternion(x, y, z, w):
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return yaw_z # in radians

class pure_pursuit_class(Node) :
    def __init__(self):
        super().__init__('pure_pursuit_node_test')
        self.timer = self.create_timer(sampling_time, self.timer_callback)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription1 = self.create_subscription(
            Path,
            '/nav_path',
            self.listener_callback,
            200)

        self.subscription3 = self.create_subscription(
            Odometry,
            '/odom',
            self.feedback_callback,
            30)
        
        self.pub_msg = Twist()

        self.state = State(x=0.0, y=.0, yaw=0.0, v=0.0)
        self.target_speed = 0.0 # [m/s]
        self.lastIndex = 1
        self.target_ind = 0
        self.flag = 0
        self.ema_v = 0.0
        self.ema_v_old = 0.0
        
    def timer_callback(self):

        if self.flag == 1 : 

            input_v = 0.25
            if (input_v >= self.target_speed):
                self.target_speed = alpha * input_v + (1 - alpha) * self.ema_v_old
            self.ema_v_old = self.target_speed
            
            self.di, self.target_ind = pure_pursuit_steer_control(
                self.state, self.target_course, self.target_ind)
            self.pub_msg.angular.z = self.di

            print("speed : ", self.pub_msg.linear.x)
            print("omeaga : ", self.pub_msg.angular.z)

            if self.lastIndex <= self.target_ind and np.linalg.norm([self.state.x - self.path_x[-1], self.state.y - self.path_y[-1]]) < Lfc :
                error  = (self.path_x[-1] - self.state.x) * math.cos(self.state.yaw) +  (self.path_y[-1] - self.state.y) * math.sin(self.state.yaw)
                self.target_speed = Kp_pos * error
                print("........")
                
                if self.target_speed < 0.001 and self.target_speed > -0.001:
                    self.target_speed = 0.0
                

                if error < 0.1 :
                    print("complete")
                    self.target_speed = 0.0
                    self.pub_msg.angular.z = 0.0
                    self.lastIndex = 1
                    self.target_ind = 0

                    self.flag = 0             

        # self.ai = proportional_control(self.target_speed, self.state.v)
        
        self.pub_msg.linear.x = self.target_speed
        self.publisher_.publish(self.pub_msg)

    def listener_callback(self, path_msg):
        if self.flag == 0 :
            self.flag = 1
            self.path_x, self.path_y = [], []
            x = [pose.pose.position.x for pose in path_msg.poses]
            y = [pose.pose.position.y for pose in path_msg.poses]
            self.path_x.extend(x)
            self.path_y.extend(y)

            self.lastIndex = len(self.path_x) - 1
            self.target_course = TargetCourse(self.path_x, self.path_y)
            self.target_ind, _ = self.target_course.search_target_index(self.state)
            


    def feedback_callback(self, msg):
        # msg = Odometry()
        current_velocity = msg.twist.twist.linear.x
        self.current_x = (float)(msg.pose.pose.position.x)
        self.current_y = (float)(msg.pose.pose.position.y)
        self.current_yaw = (float)(euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))
        self.current_state = [self.current_x, self.current_y, self.current_yaw]
        self.state = State(x=self.current_x, y=self.current_y, yaw=self.current_yaw, v = current_velocity)




def main(args=None):
    rclpy.init(args=args)

    pure_pursuit_class_ = pure_pursuit_class()

    rclpy.spin(pure_pursuit_class_)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pure_pursuit_class_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()