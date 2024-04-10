import struct
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
import tf_transformations # TODO: make sure this is installed on the cars
from collections import defaultdict
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import socket
from time import sleep, time
import json
import threading
import numpy as np
import math
from geopy.distance import geodesic

MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4
ABORT = -1

NUM_CARS = 1 # number of total cars in the network, including the ego vehicle
EARTH_RADIUS = 6371e3 # earth radius in meters
# TODO: Tune params below
KP = 0.5
KD = 0.5
K = 0.5
BROADCAST_INTERVAL = 1
LISTEN_INTERVAL = 0.01
MAX_STEER = np.pi/4
WHEELBASE = 1.0
CAR_LENGTH = 1.0
FOLLOW_DISTANCE = 2.0 # meters behind the immediate preceding vehicle, 4 meters behind the second preceding vehicle, etc.

class UDPPublisher(Node):
	def __init__(self, car):
		super().__init__('udp_publisher')

        # setup related to udp communication
		self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		ttl = struct.pack('b', 1)
		self.broadcast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

		self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.listen_sock.bind(('', 5004))
		group = socket.inet_aton('224.0.0.1')
		mreq = struct.pack('4sL', group, socket.INADDR_ANY)
		self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

		interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
		self.allips = [ip[-1][0] for ip in interfaces]

		self.broadcast_timer = self.create_timer(BROADCAST_INTERVAL, self.broadcast_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
		self.listen_timer = self.create_timer(LISTEN_INTERVAL, self.listen_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
		self.car = car
		self.car_positions = defaultdict(list)

		# setup related to position
		# self.telem_subscription = self.create_subscription(Odometry, '/mavros/global_position/local', self.telem_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.satellite = None
		self.satellite_msgs = []
		self.telem_subscription = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.telem_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.satellite_subscriber = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.satellite_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.velocity_subscriber = self.create_subscription(TwistStamped, '/mavros/global_position/gp_vel', self.velocity_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())

	def broadcast_timer_callback(self):
		"""Broadcasts the car's current GPS position to all other cars in the network"""

		# if we haven't received a GPS position yet, don't broadcast
		if self.satellite is None:
			return

		# velocity_mag = np.sqrt(self.velocity.twist.linear.x**2 + self.velocity.twist.linear.y**2)
		msg = json.dumps({"car": self.car, "lat": self.satellite.latitude, "lon": self.satellite.longitude, "time": time()})
		msg = msg.encode()

		self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

	def listen_timer_callback(self):
		"""Listens for GPS positions from other cars in the network, converts them to the local frame of the car, and stores them in a dictionary."""

		data, _ = self.listen_sock.recvfrom(1024)
		data_json = json.loads(data.decode())

		if data_json['car'] <= self.car:
			try:
				current_lat = self.satellite.latitude
				current_lon = self.satellite.longitude
			except:
				return
			
			distance = geodesic((current_lat, current_lon), (data_json['lat'], data_json['lon'])).meters
			print(f"Distance between cars {self.car} and {data_json['car']}: {distance:.2f} meters")

			current_xyz = np.array([self.telem.pose.position.x, self.telem.pose.position.y, 0])
			# current_quaternion = np.array([self.telem.pose.pose.orientation.x, self.telem.pose.pose.orientation.y, 0, self.telem.pose.pose.orientation.w])
			current_quaternion = np.array([self.telem.pose.orientation.x, self.telem.pose.orientation.y, self.telem.pose.orientation.z, self.telem.pose.orientation.w])

			# Convert lat, lon to radians
			target_lat_rad, target_lon_rad = math.radians(data_json['lat']), math.radians(data_json['lon'])
			current_lat_rad, current_lon_rad = math.radians(current_lat), math.radians(current_lon)

			# Equirectangular projection
			x = EARTH_RADIUS * (target_lon_rad - current_lon_rad) * math.cos((current_lat_rad + target_lat_rad) / 2)
			y = EARTH_RADIUS * (target_lat_rad - current_lat_rad)

			# Relative position in global frame
			relative_position = np.array([x, y, 0])

			# Convert quaternion to rotation matrix
			rotation_matrix = Rotation.from_quat(current_quaternion).as_matrix()

			# Rotate relative position into local frame
			local_position = rotation_matrix @ relative_position

			local_position_translated = local_position + np.array(current_xyz)

			position_update = local_position_translated[:2] + [data_json['time']] # only store x, y, and time of transmission
			
			if data_json['car'] == 0:
				print(f"ego_position: x: {self.telem.pose.orientation.x:.5f}, y: {self.telem.pose.orientation.y:.5f}, z: {self.telem.pose.orientation.z:.5f}")
				print(f"relative car position: x: {relative_position[0]:.5f}, y: {relative_position[1]:.5f}, z: {relative_position[2]:.5f}")
				print(f"local car position: x: {local_position[0]:.5f}, y: {local_position[1]:.5f}, z: {local_position[2]:.5f}")
				print(f"local car position translated: x: {local_position_translated[0]:.5f}, y: {local_position_translated[1]:.5f}, z: {local_position_translated[2]:.5f}")


			self.car_positions[data_json['car']].append(position_update)
			self.car_positions[data_json['car']] = self.car_positions[data_json['car']].copy()[-2:] # only store the last 2 positions
			
			if data_json['car'] == 0 and len(self.car_positions[0]) == 2:
				x1, y1, time1 = self.car_positions[0][0]
				x2, y2, time2 = self.car_positions[0][1]
				print(f"lead vehicle most recent update:\n\tx:\ty:\n{time1}\t{x1}\t{y1}\n{time2}\t{x2}\t{y2}\n")
                
	def telem_listener_callback(self, msg):
		"""Saves the latest telemetry message"""
		self.telem = msg

	def satellite_listener_callback(self, msg):
		"""Saves the latest GPS message"""
		self.satellite = msg

	def velocity_listener_callback(self, msg):
		"""Saves the latest velocity message"""
		self.velocity = msg

rclpy.init(args=None)
udp_publisher = UDPPublisher(int(input()))
executor = MultiThreadedExecutor()
executor.add_node(udp_publisher)
executor.spin()