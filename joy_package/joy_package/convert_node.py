import rclpy
from rclpy.node import Node

# Ros 2 message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# Parameters
N = 15 # Number of periods for the EMA
alpha = 2 / (N + 1)

class movement(Node):
    def __init__(self):
        super().__init__('control_node')
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 20)
        self.twist_pub = self.create_publisher  (Twist, '/cmd_vel_joy',20)
        self.timer = self.create_timer(1/20, self.timer_callback)

        self.ema_v_old = 0.0
        self.ema_v = 0.0

        self.vx = 0.0
        self.omega = 0.0
        self.ema_omega = 0.0
        self.ema_omega_old = 0.0
        self.has_controller = False
    
    def joy_callback(self, msg):
        self.has_controller = True
        self.vx = msg.axes[1]
        self.omega = msg.axes[3]
        
    
    def timer_callback(self):
        if self.has_controller == True :
            twist = Twist()
            input_v = 0.5 * self.vx
            input_omega = self.omega
            
            self.ema_v = alpha * input_v + (1 - alpha) * self.ema_v_old
            self.ema_omega = alpha * input_omega + (1 - alpha) * self.ema_omega_old
            
            if self.ema_v < 0.001 and self.ema_v > -0.001:
                self.ema_v = 0.0
            if self.ema_omega < 0.001 and self.ema_omega > -0.001:
                self.ema_omega = 0.0

            twist.linear.x = self.ema_v
            twist.angular.z = self.ema_omega


            self.twist_pub.publish(twist)

            self.ema_omega_old = self.ema_omega
            self.ema_v_old = self.ema_v
        

def main(args=None):
    rclpy.init(args=args)
    movement_node = movement()
    rclpy.spin(movement_node)
    movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()