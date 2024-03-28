import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import NavSatFix
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from scipy.spatial.transform import Rotation
import socket
from time import sleep, time
import json
import threading
import numpy as np
import math

class UDPPublisher(Node):
	def __init__(self, car):
		super().__init__('udp_publisher')

        # setup related to broadcasting
		self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
		self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
		self.client_sock.bind(("", 37020))

		interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
		self.allips = [ip[-1][0] for ip in interfaces]

		self.broadcast_timer = self.create_timer(1, self.broadcast_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

		# setup related to position
		self.satellite_subscriber = self.create_subscription(NavSatFix, '/fix', self.satellite_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())

	def broadcast_timer_callback(self):
		"""Broadcasts the car's current GPS position to all other cars in the network"""

		# if we haven't received a GPS position yet, don't broadcast
		if self.satellite is None:
			return

		msg = json.dumps({"car": self.car, "lat": self.satellite.latitude, "lon": self.satellite.longitude})
		msg = msg.encode()

		for ip in self.allips:
			# print(f'sending on {ip}')
			sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
			sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
			sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
			sock.bind((ip,0))
			sock.sendto(msg, ("255.255.255.255", 37020))
			sock.close()
			
	def satellite_listener_callback(self, msg):
		"""Callback function for receiving GPS data"""
		self.satellite = msg
	

rclpy.init(args=None)
udp_publisher = UDPPublisher(0)
executor = MultiThreadedExecutor()
executor.add_node(udp_publisher)
executor.spin()