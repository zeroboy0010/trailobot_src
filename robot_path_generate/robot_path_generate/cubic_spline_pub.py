import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

try :
    from robot_path_generate import cubic_spline_planner
except :
    pass
try :
    import cubic_spline_planner
except :
    pass

def get_straight_course(dl):      # use for two point
    ax = [0.0 ,2.0, 2.0, 4.0]
    ay = [0.0, 0.0, 2.0, 2.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.publisher_ = self.create_publisher(Path, '/generated_path', 200)
        self.timer = self.create_timer(1.0, self.timer_callback)  # publish every second
        self.path = Path()
        self.path.header = Header()
        self.path.header.frame_id = 'odom'  # Set the coordinate frame of the path
        self.get_logger().info('Path generator node has been started.')

    

        cx, cy, cyaw, ck = get_straight_course(0.2)
        num_points = len(cx)
        for i in range(len(cx)):
            pose = PoseStamped()
            pose.header = self.path.header
            pose.pose.position.x = cx[i]
            pose.pose.position.y = cy[i]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            self.path.poses.append(pose)
        
        # Update the timestamp
        self.path.header.stamp = self.get_clock().now().to_msg()

        # Publish the path
        self.publisher_.publish(self.path)
        self.get_logger().info('Publishing path with {} points'.format(num_points))

    def timer_callback(self):
        # Generate a simple straight-line path for demonstration
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
