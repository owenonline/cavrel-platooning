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

measurements = []

point = None
vel = None
i = time()

class UDPPublisher(Node):
	def __init__(self, car):
		super().__init__('udp_publisher')

		# setup related to position
		# self.telem_subscription = self.create_subscription(Odometry, '/mavros/global_position/local', self.telem_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.satellite_subscriber = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.satellite_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.velocity_subscriber = self.create_subscription(TwistStamped, '/mavros/global_position/gp_vel', self.velocity_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.timer = self.create_timer(0.1, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

	def satellite_listener_callback(self, data):
		global point
		point = data
		# points.append([data.latitude, data.longitude])
		# points_np = np.array(points)
		# mean_lat = np.mean(points_np[:,0])
		# mean_lon = np.mean(points_np[:,1])
		# std_lat = np.std(points_np[:,0])
		# std_lon = np.std(points_np[:,1])
		# print(("latitude: "+str(std_lat)+", Longitude: "+str(std_lon)))
  
	def velocity_listener_callback(self, data):
		global vel
		vel = data

	def timer_callback(self):
		global point, vel, measurements, i
		if point is None or vel is None:
			return
		measurements.append([point.latitude, point.longitude, vel.twist.linear.x, vel.twist.linear.y])
		if time()-i > 10:
			with open("measurements.npy", "wb") as f:
				np.save(f, np.array(measurements))
			raise Exception("Done")

rclpy.init(args=None)
udp_publisher = UDPPublisher(int(input()))
executor = MultiThreadedExecutor()
executor.add_node(udp_publisher)
executor.spin()