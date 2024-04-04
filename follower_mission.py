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
from scipy.optimize import minimize
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

class UDPPublisher(Node):
	def __init__(self, car):
		super().__init__('udp_publisher')

        # setup related to udp communication
		self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
		self.broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
		self.broadcast_sock.bind(("", 37020))
		self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.listen_sock.bind(("", 37020))

		interfaces = socket.getaddrinfo(host=socket.gethostname(), port=None, family=socket.AF_INET)
		self.allips = [ip[-1][0] for ip in interfaces]

		self.broadcast_timer = self.create_timer(BROADCAST_INTERVAL, self.broadcast_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
		self.listen_timer = self.create_timer(LISTEN_INTERVAL, self.listen_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
		self.car = car
		self.car_positions = defaultdict(list)

		# setup related to position
		self.telem_subscription = self.create_subscription(Odometry, '/mavros/global_position/local', self.telem_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.satellite_subscriber = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.satellite_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.velocity_subscriber = self.create_subscription(TwistStamped, '/mavros/global_position/gp_vel', self.velocity_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		
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
		self.mission_timer = self.create_timer(0.1, self.mission_timer_callback)

		# setup kill switch
		self.stop_thread = threading.Thread(target=self.listen_for_stop)
		self.stop_thread.daemon = True
		self.stop_thread.start()

		# setup mission state
		self.mission_status = MISSIONSTART

	def listen_for_stop(self):
		"""Listens for a KILL command from the user to abort the mission immediately."""
		while True:
			command = input()
			if command.upper() == 'KILL':
				self.mission_status = ABORT

	def broadcast_timer_callback(self):
		"""Broadcasts the car's current GPS position to all other cars in the network"""

		# if we haven't received a GPS position yet, don't broadcast
		if self.satellite is None:
			return

		# velocity_mag = np.sqrt(self.velocity.twist.linear.x**2 + self.velocity.twist.linear.y**2)
		msg = json.dumps({"car": self.car, "lat": self.satellite.latitude, "lon": self.satellite.longitude, "time": time(), "abort": self.mission_status == ABORT})
		msg = msg.encode()

		self.broadcast_sock.sendto(msg, ("255.255.255.255", 37020))

	def listen_timer_callback(self):
		"""Listens for GPS positions from other cars in the network, converts them to the local frame of the car, and stores them in a dictionary."""

		data, _ = self.listen_sock.recvfrom(1024)
		data_json = json.loads(data.decode())

		# if one of the cars failed, stop the mission immediately
		if data_json['abort']:
			self.mission_status = ABORT

		if data_json['car'] <= self.car:
			current_lat = self.satellite.latitude
			current_lon = self.satellite.longitude
			current_xyz = np.array([self.telem.pose.pose.position.x, self.telem.pose.pose.position.y, 0])
			current_quaternion = np.array([self.telem.pose.pose.orientation.x, self.telem.pose.pose.orientation.y, 0, self.telem.pose.pose.orientation.w])

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
			self.car_positions[data_json['car']].append(position_update)
			self.car_positions[data_json['car']] = self.car_positions[data_json['car']].copy()[-2:] # only store the last 2 positions

	def telem_listener_callback(self, msg):
		"""Saves the latest telemetry message"""
		self.telem = msg

	def satellite_listener_callback(self, msg):
		"""Saves the latest GPS message"""
		self.satellite = msg

	def velocity_listener_callback(self, msg):
		"""Saves the latest velocity message"""
		self.velocity = msg

	def get_goal_motion(self):
		targets = []
		for i in range(self.car - 1, -1, -1):
			(x1, y1, time1), (x2, y2, time2) = self.car_positions[i]

			velocity = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)/(time2 - time1)
			heading = np.arctan2(y2 - y1, x2 - x1)
			targets.append((x2, y2, heading, velocity))

		def minimization_objective(params):
			v, head = params
			x, y = self.telem.pose.pose.position.x, self.telem.pose.pose.position.y

			total_cost = 0
			for target in targets:
				x_target, y_target, head_target, v_target = target
				total_cost += np.sqrt((x_target + v_target*np.cos(head_target) - x - v*np.cos(head))**2 + (y_target + v_target*np.sin(head_target) - y - v*np.sin(head))**2)

			return total_cost
		
		res = minimize(minimization_objective, [1.0, 0.0])

		return res, targets
	
	def pd_controller(self, v, v_ego):
		accel = KP*(v - v_ego) + KD*(v - v_ego)/BROADCAST_INTERVAL
		return accel
	
	def stanley_controller(self, x_ego, y_ego, head_ego, v_ego, head, target_x, target_y):
		e = abs((x_ego - target_x) * np.sin(head) - (y_ego - target_y) * np.cos(head))
		steer = np.arctan2(K*e/v_ego)+(head_ego - head)

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

			if time() - self.start_time > 5:
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
			else:
				# get the position, heading, and velocity for each preceding vehicle, then minimize to get the target speed and heading
				res, targets = self.get_goal_motion()				

				if not res.success:
					self.mission_status = ABORT
					return
				
				v, head = res.x

				# get the motion of the ego vehicle
				v_ego = np.sqrt(self.velocity.twist.linear.x**2 + self.velocity.twist.linear.y**2)
				_, _, head_ego = tf_transformations.euler_from_quaternion([self.telem.pose.pose.orientation.x, self.telem.pose.pose.orientation.y, self.telem.pose.pose.orientation.z, self.telem.pose.pose.orientation.w])
				x_ego, y_ego = self.telem.pose.pose.position.x, self.telem.pose.pose.position.y

				# update velocity and yaw
				accel = self.pd_controller(v, v_ego)
				steer = self.stanley_controller(x_ego, y_ego, head_ego, v_ego, head, targets[0][0], targets[0][1])
				steer = np.clip(steer, -MAX_STEER, MAX_STEER)
				head_ego += v_ego/WHEELBASE*np.tan(steer)*BROADCAST_INTERVAL
				v_ego += accel*BROADCAST_INTERVAL

				# send updated control message
				msg.linear.x = v_ego*np.cos(head_ego)
				msg.linear.y = v_ego*np.sin(head_ego)
				msg.linear.z = 0.0

				msg.angular.x = 0.0
				msg.angular.y = 0.0
				msg.angular.z = 0.0

				self.publisher.publish(msg)
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
		print("...disarmed, mission complete")
		self.mission_status = MISSIONCOMPLETE

rclpy.init(args=None)
udp_publisher = UDPPublisher(int(input()))
executor = MultiThreadedExecutor()
executor.add_node(udp_publisher)
executor.spin()