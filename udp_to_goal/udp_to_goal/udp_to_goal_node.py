# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# import socket

# class UDPToGoalNode(Node):
#     def __init__(self):
#         super().__init__('udp_to_goal_node')
#         self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

#         # Set up UDP server
#         self.udp_ip = self.get_own_ip_address()
#         self.udp_port = 5005
#         self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.udp_socket.bind((self.udp_ip, self.udp_port))

#         # Start listening for UDP messages
#         self.timer = self.create_timer(0.1, self.receive_udp_message)

#     def get_own_ip_address(self):
#         try:
#             # Create a dummy socket connection to get the machine's IP address
#             s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#             s.connect(('8.8.8.8', 80))  # Google DNS server, can be any reachable IP
#             ip_address = s.getsockname()[0]
#             self.get_logger().info(f"IP address: {ip_address}")
#             s.close()
#             return ip_address
#         except Exception as e:
#             self.get_logger().error(f"Unable to retrieve IP address: {e}")
#             return '127.0.0.1'  # Fallback to localhost
#     def receive_udp_message(self):
#         try:
#             data, addr = self.udp_socket.recvfrom(1024)  # Buffer size 1024 bytes
#             self.get_logger().info(f"Received message from {addr}: {data.decode()}")
            
#             # Assuming data is in format "x,y,z" for Pose
#             try:
#                 x, y, z = map(float, data.decode().split(','))
#             except ValueError:
#                 self.get_logger().error('Received invalid data')
#                 return

#             # Create PoseStamped message
#             goal_pose = PoseStamped()
#             goal_pose.header.stamp = self.get_clock().now().to_msg()
#             goal_pose.header.frame_id = 'map'  # Adjust if necessary
#             goal_pose.pose.position.x = x
#             goal_pose.pose.position.y = y
#             goal_pose.pose.position.z = 0.0
#             goal_pose.pose.orientation.w = 1.0  # Default orientation

#             # Publish the goal pose
#             self.publisher.publish(goal_pose)
#             self.get_logger().info(f"Published goal pose: {goal_pose.pose.position}")

#         except socket.error as e:
#             self.get_logger().error(f"Socket error: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = UDPToGoalNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import serial

class SerialNode(Node):
	def __init__(self):
		super().__init__('serial_node')
		self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
		self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
		self.buffer = ""
		self.timer = self.create_timer(0.1, self.read_serial_data)

	def read_serial_data(self):
		if self.serial_port.in_waiting > 0:
			try:
				data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
				self.buffer += data
				self.get_logger().info(f"Buffer: {self.buffer}")
				
				while "Data received" in self.buffer:
					start_idx = self.buffer.find("Data received")
					end_idx = self.buffer.find("Data received", start_idx + 1)
					
					if end_idx == -1:
						break
					
					complete_message = self.buffer[start_idx:end_idx].strip()
					self.buffer = self.buffer[end_idx:]
					
					self.process_message(complete_message)
			except Exception as e:
				self.get_logger().error(f"Error reading serial data: {e}")

	def process_message(self, message):
		self.get_logger().info(f"Processing message: {message}")
		lines = message.split('\n')
		pose = PoseStamped()
		pose.header.frame_id = 'map'
		pose.header.stamp = self.get_clock().now().to_msg()
		for line in lines:
			self.get_logger().info(f"Processing line: {line}")
			if line.startswith("x:"):
				pose.pose.position.x = float(line.split(':')[1].strip())
			elif line.startswith("y:"):
				pose.pose.position.y = float(line.split(':')[1].strip())
			elif line.startswith("z:"):
				pose.pose.position.z = float(line.split(':')[1].strip())
		self.publisher_.publish(pose)
		self.get_logger().info(f"Published pose: {pose}")

def main(args=None):
	rclpy.init(args=args)
	node = SerialNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()