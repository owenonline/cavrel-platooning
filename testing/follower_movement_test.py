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
from scipy.optimize import minimize, least_squares, Bounds
from scipy.stats import linregress
import pyproj
import socket
from time import sleep, time
import json
import threading
import numpy as np
import math

MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4
ABORT = -1

NUM_CARS = 2 # number of total cars in the network, including the ego vehicle
EARTH_RADIUS = 6371e3 # earth radius in meters
# TODO: Tune params below
KPV = 0.3
KDV = 0.5
K = 0.5
BROADCAST_INTERVAL = 0.1 # same for all cars
LISTEN_INTERVAL = 0.01
MAX_STEER = 30
WHEELBASE = 0.48
CAR_LENGTH = 0.779
FOLLOW_DISTANCE = 2.0 # meters behind the immediate preceding vehicle, 4 meters behind the second preceding vehicle, etc.
DUE_EAST = 90
SPEED_LIMIT = 0.05
geodesic = pyproj.Geod(ellps='WGS84')
center_latitude = (28.607980 + 28.607292) / 2
center_longitude = (-81.195662 + -81.194750) / 2
center_orientation = DUE_EAST

class UDPPublisher(Node):
	def __init__(self, speed):
		super().__init__('udp_publisher')

        # setup related to motion
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
		self.mission_timer = self.create_timer(LISTEN_INTERVAL, self.mission_timer_callback)

		self.heading = None
		self.heading_subscriber = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())

		# setup input handler
		self.stop_thread = threading.Thread(target=self.listen_for_stop)
		self.stop_thread.daemon = True
		self.stop_thread.start()

		# setup mission state
		self.vset = speed

		self.mission_status = MISSIONSTART

	def heading_listener_callback(self, msg):
		"""Saves the latest heading message"""
		self.heading = msg

	def listen_for_stop(self):
		"""Listens for a KILL command from the user to abort the mission immediately."""
		while True:
			command = input()
			if command == 'k':
				self.mission_status = ABORT
			
	def mission_timer_callback(self):
		"""Main loop for vehicle control. Handles the arming, moving, and disarming of the rover."""

		# TODO: Make sure this waits for the cars in front of the current car rather than all of the cars, or some other heuristic
		# wait until we've received position data from all the other cars before starting the mission
		# if len(list(self.car_positions.values())) < NUM_CARS - 1:
		# 	return

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

			speed = 0.3

			if time() - self.start_time < 5:
				print("no movement")
				msg.linear.x = 0.0
				msg.linear.y = 0.0
				msg.linear.z = 0.0
				msg.angular.x = 0.0
				msg.angular.y = 0.0
				msg.angular.z = 0.0

				self.publisher.publish(msg)
			elif time() - self.start_time < 10:
				print("heading at 345 degrees")

				angle = np.radians(345)

				msg.linear.x = speed * np.cos(angle)
				msg.linear.y = speed * np.sin(angle)
				msg.linear.z = 0.0
				msg.angular.x = 0.0
				msg.angular.y = 0.0
				msg.angular.z = 0.0

				self.publisher.publish(msg)
			elif time() - self.start_time < 15:
				print("heading at 20 degrees")

				angle = np.radians(20)

				msg.linear.x = speed * np.cos(angle)
				msg.linear.y = speed * np.sin(angle)
				msg.linear.z = 0.0
				msg.angular.x = 0.0
				msg.angular.y = 0.0
				msg.angular.z = 0.0

				self.publisher.publish(msg)
			elif time() - self.start_time < 20:
				print("heading at 300 degrees")

				angle = np.radians(300)

				msg.linear.x = speed * np.cos(angle)
				msg.linear.y = speed * np.sin(angle)
				msg.linear.z = 0.0
				msg.angular.x = 0.0
				msg.angular.y = 0.0
				msg.angular.z = 0.0

				self.publisher.publish(msg)
			else:
				print("...stopped, disarming")
				msg.linear.x = 0.0
				msg.linear.y = 0.0
				msg.linear.z = 0.0

				msg.angular.x = 0.0
				msg.angular.y = 0.0
				msg.angular.z = 0.0
				
				self.publisher.publish(msg)

				disarm_req = CommandBool.Request()
				disarm_req.value = False
				disarm_future = self.arming_client.call_async(disarm_req)
				disarm_future.add_done_callback(self.disarm_callback)

				self.mission_status = DISARMING
			return
		
		
		if self.mission_status == DISARMING:
			msg = Twist()
			msg.linear.x = 0.0
			msg.linear.y = 0.0
			msg.linear.z = 0.0

			msg.angular.x = 0.0
			msg.angular.y = 0.0
			msg.angular.z = 0.0
			
			self.publisher.publish(msg) # continue publishing stop messages until disarmed
			return
	
		if self.mission_status == MISSIONCOMPLETE:
			print("MISSION COMPLETE")
			self.log_handle.close()
			self.destroy_node()
			rclpy.shutdown()
		
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

rclpy.init(args=None)
udp_publisher = UDPPublisher(float(input()))
executor = MultiThreadedExecutor()
executor.add_node(udp_publisher)
executor.spin()