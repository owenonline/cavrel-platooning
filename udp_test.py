import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import socket
from time import sleep
import json

class UDPPublisher(Node):
	def __init__(self, car):
		super().__init__('udp_publisher')

		self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
		self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
		self.client_sock.bind(("", 37020))

		interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
		self.allips = [ip[-1][0] for ip in interfaces]

		self.broadcast_timer = self.create_timer(1, self.broadcast_timer_callback)
		self.listen_timer = self.create_timer(0.001, self.listen_timer_callback)
		self.car = car
		self.i = 0
		self.received_messages = []

	def broadcast_timer_callback(self):
		msg = json.dumps({"car": self.car, "msg": self.i})
		msg = msg.encode()

		for ip in self.allips:
			# print(f'sending on {ip}')
			sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
			sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
			sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
			sock.bind((ip,0))
			sock.sendto(msg, ("255.255.255.255", 37020))
			sock.close()

		self.i += 1

	def listen_timer_callback(self):
		data, addr = self.client_sock.recvfrom(1024)
		data_json = json.loads(data.decode())
		if data_json['car'] != self.car and not data_json['msg'] in self.received_messages:
			print(f"received message {data_json['msg']} from car: {data_json['car']}")
			self.received_messages.append(data_json['msg'])


rclpy.init(args=None)
udp_publisher = UDPPublisher(int(input()))
executor = MultiThreadedExecutor()
executor.add_node(udp_publisher)
executor.spin()