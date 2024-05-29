import random
import struct
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped
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
from scipy.optimize import minimize, least_squares, Bounds, minimize_scalar
from scipy.stats import linregress
from scipy.interpolate import CubicSpline
from datetime import datetime
import pickle
import pyproj
import socket
from time import sleep, time
import json
import threading
import numpy as np
import math
import os
import argparse

# set up mission states
MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4
ABORT = -1

# set up constants
MAX_STEER = 30
WHEELBASE = 0.48
CAR_LENGTH = 0.779
FOLLOW_DISTANCE = 2.0 # meters behind the immediate preceding vehicle, 4 meters behind the second preceding vehicle, etc.
DUE_EAST = 90
SPEED_LIMIT = 10
geodesic = pyproj.Geod(ellps='WGS84')

class UDPPublisher(Node):
	def __init__(self):
		super().__init__('udp_publisher')

		self.mission_status = MISSIONSTART
		self.pressed_keys = set()

		self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.listen_sock.bind(('', 5004))
		group = socket.inet_aton('224.0.0.1')
		mreq = struct.pack('4sL', group, socket.INADDR_ANY)
		self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
		self.listen_timer = self.create_timer(0.001, self.listen_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

		# publisher setup
		print('setting up motion clients')
		self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
		while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
			print('set mode service not available, waiting again...')
			
		self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
		while not self.arming_client.wait_for_service(timeout_sec=1.0):
			print('arming service not available, waiting again...')

		self.killswitch_client = self.create_client(CommandLong, '/mavros/cmd/command')
		while not self.killswitch_client.wait_for_service(timeout_sec=1.0):
			print('killswitch service not available, waiting again...')

		self.publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 20)
		self.mission_timer = self.create_timer(0.1, self.mission_timer_callback)

	def listen_timer_callback(self):
		"""Listens for and stores the latest broadcasts from other cars in the network."""

		data, _ = self.listen_sock.recvfrom(1024)
		data_json = json.loads(data.decode())

		self.pressed_keys = set(data_json['keys'])

	def mission_timer_callback(self):
		"""Main loop for vehicle control. Handles the arming, moving, and disarming of the rover."""

		# start the chain of events that arms the rover
		if self.mission_status == MISSIONSTART:
			print("switching to offboard mode")
			set_mode_req = SetMode.Request()
			set_mode_req.base_mode = 0
			set_mode_req.custom_mode = "OFFBOARD"
			set_mode_future = self.set_mode_client.call_async(set_mode_req)
			set_mode_future.add_done_callback(self.init_callback)
			self.mission_status = ARMING
			return
		
		if self.mission_status == ARMING:
			# px4 requires a stream of setpoint messages to be sent to the rover in order to arm
			msg = Twist()
			msg.linear.x = 0.0
			msg.linear.y = 0.0
			msg.linear.z = 0.0

			msg.angular.x = 0.0
			msg.angular.y = 0.0
			msg.angular.z = 0.0
			
			self.publisher.publish(msg)
			return
		
		if self.mission_status == MOVING:
			msg = Twist()

			new_speed = 0
			delta = 0

			if 'w' in self.pressed_keys:
				new_speed = SPEED_LIMIT
			elif 's' in self.pressed_keys:
				new_speed = -SPEED_LIMIT

			if 'a' in self.pressed_keys:
				delta = MAX_STEER
			elif 'd' in self.pressed_keys:
				delta = -MAX_STEER

			if 'p' in self.pressed_keys:
				disarm_req = CommandBool.Request()
				disarm_req.value = False
				disarm_future = self.arming_client.call_async(disarm_req)
				disarm_future.add_done_callback(self.disarm_callback)

				self.mission_status = DISARMING
				return
		
			delta = math.radians(delta)

			msg.linear.x = -new_speed * math.sin(delta)
			msg.linear.y = new_speed * math.cos(delta)
			msg.linear.z = 0.0
			msg.angular.x = 0.0
			msg.angular.y = 0.0
			msg.angular.z = 0.0

			print(msg.linear.x, msg.linear.y, msg.angular.z)

			self.publisher.publish(msg)
			return
	
		if self.mission_status == MISSIONCOMPLETE:
			print("MISSION COMPLETE")
			self.destroy_node()
			rclpy.shutdown()

		if self.mission_status == DISARMING:
			msg = Twist()

			msg.linear.x = 0.0
			msg.linear.y = 0.0
			msg.linear.z = 0.0
			msg.angular.x = 0.0
			msg.angular.y = 0.0
			msg.angular.z = 0.0
			self.publisher.publish(msg)
			return
		
		# if the mission is aborted, turn off all motors immediately.
		if self.mission_status == ABORT:
			print("ABORTING")

			emergency_disarm_req = CommandLong.Request()
			emergency_disarm_req.broadcast = False
			emergency_disarm_req.command = 400
			emergency_disarm_req.confirmation = 0
			emergency_disarm_req.param1 = 0.0
			emergency_disarm_req.param2 = 21196.0
			emergency_disarm_req.param3 = 0.0
			emergency_disarm_req.param4 = 0.0
			emergency_disarm_req.param5 = 0.0
			emergency_disarm_req.param6 = 0.0
			emergency_disarm_req.param7 = 0.0

			self.killswitch_client.call_async(emergency_disarm_req)
			self.mission_status = MISSIONCOMPLETE
			return
		
	def init_callback(self, future):
		print("...in offboard mode, arming")
		sleep(4) # wait for the stream of messages to be long enough to allow arming
		arm_req = CommandBool.Request()
		arm_req.value = True
		arm_future = self.arming_client.call_async(arm_req)
		arm_future.add_done_callback(self.arm_callback)

	def arm_callback(self, future):
		print("...armed, moving")
		self.start_time = time()
		self.mission_status = MOVING

	def disarm_callback(self, future):
		print("...disarmed")
		self.mission_status = MISSIONCOMPLETE

if __name__ == '__main__':
	rclpy.init(args=None)
	udp_publisher = UDPPublisher()
	executor = MultiThreadedExecutor()
	executor.add_node(udp_publisher)
	executor.spin()