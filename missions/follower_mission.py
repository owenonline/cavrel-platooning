import random
import struct
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
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
EARTH_RADIUS = 6371e3
KPV = 0.3
KDV = 0.5
K = 0.2
LISTEN_INTERVAL = 0.01
MAX_STEER = 30
WHEELBASE = 0.48
CAR_LENGTH = 0.779
FOLLOW_DISTANCE = 2.0 # meters behind the immediate preceding vehicle, 4 meters behind the second preceding vehicle, etc.
DUE_EAST = 90
SPEED_LIMIT = 2.2
geodesic = pyproj.Geod(ellps='WGS84')

# set up args
parser = argparse.ArgumentParser()
parser.add_argument('--track_path', type=str, default="missions/tracks/bus_loop_large.json")
parser.add_argument('--broadcast_int', type=float, default=0.1)
parser.add_argument('--drop_rate', type=float, default=0.0) # try up thru 0.6
parser.add_argument('--car_number', type=int, default=1)

class UDPPublisher(Node):
	def __init__(self, car_number, broadcast_interval, drop_rate, center_lat, center_lon, center_orientation, track_name):
		super().__init__('udp_publisher')

		# add args to object
		self.car = car_number
		self.broadcast_interval = broadcast_interval
		self.drop_rate = drop_rate
		self.center_latitude = center_lat
		self.center_longitude = center_lon
		self.center_orientation = center_orientation
		self.track_name = track_name

		# set up starting vars
		self.datapoints = []
		self.lines = []
		self.closest = []
		self.iteration_error_count = 0 # counts how many of the vehicles in front of the ego vehicle have not sent a position update at all. This lets me start the script before i turn the other cars on so they can start moving at the same time when I turn the beacon on
		self.mission_status = MISSIONSTART
		self.telem = None
		self.satellite = None
		self.heading = None
		self.accel = None
		self.car_positions = defaultdict(list)

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

		self.broadcast_timer = self.create_timer(self.broadcast_interval, self.broadcast_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())
		self.listen_timer = self.create_timer(LISTEN_INTERVAL, self.listen_timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

		# sensor subscription setup
		self.telem_subscription = self.create_subscription(Odometry, '/mavros/global_position/local', self.telem_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.satellite_subscriber = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.satellite_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.heading_subscriber = self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())
		self.accel_subscriber = self.create_subscription(Imu, '/mavros/imu/data', self.accel_listener_callback, QoSPresetProfiles.SENSOR_DATA.value, callback_group=MutuallyExclusiveCallbackGroup())

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
		self.mission_timer = self.create_timer(self.broadcast_interval, self.mission_timer_callback)

		# setup kill switch
		self.stop_thread = threading.Thread(target=self.listen_for_stop)
		self.stop_thread.daemon = True
		self.stop_thread.start()

	def listen_for_stop(self):
		"""Kills the mission if the user presses ENTER."""

		while True:
			input()
			now = datetime.now()
			formatted_date = now.strftime('%H_%M_%d')
			
			with open(f"/home/nvidia/ros2_ws/src/cavrel-platooning/missions/datapoints/py3_{self.car}_{self.track_name}_{formatted_date}.pkl", "wb") as f:
				pickle.dump(self.datapoints, f)
			self.datapoints = None
			disarm_req = CommandBool.Request()
			disarm_req.value = False
			disarm_future = self.arming_client.call_async(disarm_req)
			disarm_future.add_done_callback(self.disarm_callback)

			self.mission_status = DISARMING

	def broadcast_timer_callback(self):
		"""Broadcasts the car's current GPS position and heading to all other cars in the network, dropping packets at a specified rate."""

		if self.satellite is None or self.heading is None or (self.drop_rate > 0 and random.random() < self.drop_rate):
			return

		msg = json.dumps({"head": self.heading.data,"car": self.car, "lat": self.satellite.latitude, "lon": self.satellite.longitude, "time": time(), "abort": self.mission_status == ABORT, "accel": (self.accel.x, self.accel.y)})
		msg = msg.encode()

		self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

	def listen_timer_callback(self):
		"""Listens for and stores the latest broadcasts from other cars in the network."""

		data, _ = self.listen_sock.recvfrom(1024)
		data_json = json.loads(data.decode())

		# if one of the cars failed, stop the mission immediately
		if data_json['abort']:
			self.mission_status = ABORT

		if data_json['car'] <= self.car:
			# save the last four positions of the car
			# we mostly use the first two for calculating current velocity, but all 4 are used for cubic spline
			# in the heading controller
			position_update = (data_json['lat'], data_json['lon'], data_json['head'], data_json['time'], data_json['accel'])
			self.car_positions[data_json['car']].append(position_update)
			self.car_positions[data_json['car']] = self.car_positions[data_json['car']][-4:] # only store the last 4 positions

	def telem_listener_callback(self, msg):
		"""Saves the latest telemetry message"""
		self.telem = msg

	def satellite_listener_callback(self, msg):
		"""Saves the latest GPS message"""
		self.satellite = msg

	def heading_listener_callback(self, msg):
		"""Saves the latest heading message"""
		self.heading = msg

	def accel_listener_callback(self, msg):
		"""Saves the latest acceleration message"""
		self.accel = msg.linear_acceleration

	def get_goal_motion(self):
		"""Calculates the target speed and heading for the ego vehicle based on the positions of the other cars in the network."""

		ex, ey = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)
		eh = self.heading.data
		ev = np.sqrt(self.telem.twist.twist.linear.x**2 + self.telem.twist.twist.linear.y**2)

		targets = []
		self.iteration_error_count = 0
		for i in range(self.car):
			if len(self.car_positions[i]) < 2:
				targets.append((ex, ey, eh, 0, self.car - i, (0,0)))
				self.iteration_error_count += 1
			else:
				(lat1, lon1, head1, time1, _), (lat2, lon2, head2, time2, accel) = self.car_positions[i][-2:]

				x1, y1 = self.coords_to_local(lat1, lon1)
				x2, y2 = self.coords_to_local(lat2, lon2)

				velocity = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)/(time2 - time1)
				heading = head2
				targets.append((x2, y2, heading, velocity, self.car - i, accel))

		# save the current state for logging later
		tmp = [(x, y, h, v, *a) for x, y, h, v, _, a in targets] + [(ex, ey, eh, ev, self.accel.x, self.accel.y)]
		self.datapoints.append(tmp)

		def minimization_objective(params):
			v, head = params
			head = np.radians(head)
			x, y = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)

			total_cost = 0
			for target in targets:
				# goal follow distance accounts for following distance and the lengths of the cars between the ego vehicle and the target vehicle
				x_target, y_target, head_target, v_target, position, _ = target
				head_target = np.radians(head_target)
				goal_follow_distance = FOLLOW_DISTANCE*position + CAR_LENGTH*(position - 1) # meters behind the target car

				# simulate the motion of the cars
				x_sim_target = x_target + v_target*np.sin(head_target)*self.broadcast_interval
				y_sim_target = y_target + v_target*np.cos(head_target)*self.broadcast_interval
				x_sim_ego = x + v*np.sin(head)*self.broadcast_interval
				y_sim_ego = y + v*np.cos(head)*self.broadcast_interval

				# determine the point where the following car *should* be to be perfectly maintaining its following distance
				x_goal = x_sim_target - goal_follow_distance*np.sin(head_target)
				y_goal = y_sim_target - goal_follow_distance*np.cos(head_target)

				# the cost is the distance between the actual simulated position of the following car and the ideal position
				total_cost += np.sqrt((x_goal - x_sim_ego)**2 + (y_goal - y_sim_ego)**2)

			return total_cost
		
		bounds = Bounds([0, -360], [10, 360])
		_, _, head, v, _, _ = targets[0]

		guesses = [[0, head], [v, head], [5, head]]
		best_score = np.inf
		best = None
		for guess in guesses:
			res = minimize(minimization_objective, guess, method='SLSQP', bounds=bounds)
			if res.fun < best_score:
				best = res

		return best
	
	def coords_to_local(self, target_lat, target_lon):
		"""Converts GPS coordinates to local cartesian coordinates with respect to the track center point."""

		target_lat_rad, target_lon_rad = math.radians(target_lat), math.radians(target_lon)
		current_lat_rad, current_lon_rad = math.radians(self.center_latitude), math.radians(self.center_longitude)

		x = EARTH_RADIUS * (target_lon_rad - current_lon_rad) * math.cos((current_lat_rad + target_lat_rad) / 2)
		y = EARTH_RADIUS * (target_lat_rad - current_lat_rad)

		angle = math.radians(self.center_orientation - DUE_EAST)
		qx = math.cos(angle) * x - math.sin(angle) * y
		qy = math.sin(angle) * x + math.cos(angle) * y
		return qx, qy
	
	def velocity_controller(self, v, v_ego):
		"""PD controller for velocity control. Computes the acceleration needed to match the target speed."""
		accel = KPV*(v - v_ego) + KDV*(v - v_ego)/self.broadcast_interval
		return accel
	
	def distance_to_line(self, x0, y0, dx, dy, x, y):
		"""Computes the distance between a point and a line."""
		lambda_val = ((x - x0) * dx + (y - y0) * dy) / (dx**2 + dy**2)
		closest_point = np.array([x0 + lambda_val * dx, y0 + lambda_val * dy])
		distance = np.linalg.norm(closest_point - np.array([x, y]))
		distance = np.nan_to_num(distance)
		return distance, closest_point
 
	def heading_controller(self, head_ego, v_ego, target_head):
		"""Stanley controller for heading control. Computes the cross track error and heading difference between the ego car and the target car.
		CURRENTLY APPROXIMATES CROSS-TRACK ERROR WITH A STRAIGHT LINE FIT."""
		points = []
		initial_guess = None

		# get the local coordinates of the last two locations of all other cars
		for i in range(self.car):
			if len(self.car_positions[i]) < 2:
				continue

			(lat1, lon1, _, _, _), (lat2, lon2, _, _, _) = self.car_positions[i][-2:]
			x1, y1 = self.coords_to_local(lat1, lon1)
			x2, y2 = self.coords_to_local(lat2, lon2)
			points.append((x1, y1))
			points.append((x2, y2))

			if i == self.car - 1: # we use the car closest to the ego vehicle as the initial guess for the line to ensure convergence
				initial_guess = [x1, y1, x2 - x1, y2 - y1]

		# if no points received, steer is 0
		if len(points) == 0 or initial_guess is None:
			return 0

		# get the local coordinates of the ego car
		lat1, lon1 = self.satellite.latitude, self.satellite.longitude
		ex1, ey1 = self.coords_to_local(lat1, lon1)

		def distances_to_line(params, points):
			x0, y0, dx, dy = params
			distances = []
			for (x, y) in points:
				distance, _ = self.distance_to_line(x0, y0, dx, dy, x, y)
				distances.append(distance)
			return distances
		
		# compute a line of best fit using the last 2 positions of all other cars
		# this allows us to accurately calculate the cross track error between where we are and where we should be to be most directly in line with the other vehicles
		# NOTE: this will start to break as you get more cars going around a curve, at which point we should switch to a curved line of best fit
		result = least_squares(distances_to_line, initial_guess, args=(points,))

		x0_opt, y0_opt, dx_opt, dy_opt = result.x
		heading_diff = target_head - head_ego

		dist, closest = self.distance_to_line(x0_opt, y0_opt, dx_opt, dy_opt, ex1, ey1)
		cte = np.arctan2(K*dist, v_ego)
		cte = np.rad2deg(cte)

		steer = heading_diff + cte
		return steer
	
	# def heading_controller(self, head_ego, v_ego, target_head):
	# 	"""Stanley controller for heading control. Computes the cross track error and heading difference between the ego car and the target car."""
	# 	points = []

	# 	# get the local coordinates of the last two locations of all other cars
	# 	for i in range(self.car):
	# 		for point in self.car_positions[i]:
	# 			(lat, lon, _, _) = point
	# 			x, y = self.coords_to_local(lat, lon)
	# 			points.append((x, y))

	# 	points.sort(key=lambda point: point[0])
	# 	xs, ys = zip(*points)

	# 	cs = CubicSpline(xs, ys)

	# 	def distance_to_spline(x):
	# 		ex, ey = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)
	# 		spline_y = cs(x)
	# 		return np.sqrt((ex - x)**2 + (ey - spline_y)**2)

	# 	res = minimize_scalar(distance_to_spline, bounds=(min(xs), max(xs)), method='bounded')
	# 	closest_x = res.x
	# 	closest_y = cs(closest_x)
	# 	dist = distance_to_spline(closest_x)

	# 	heading_diff = target_head - head_ego
	# 	cte = np.arctan2(K * dist, v_ego)
	# 	cte = np.rad2deg(cte)

	# 	line_x = np.linspace(min(xs)-2, max(xs)+2, 300)
	# 	line_y = cs(line_x)
	# 	line = (line_x, line_y)
	# 	self.lines.append(line)
	# 	self.closest.append((closest_x, closest_y))
		
	# 	steer = heading_diff + cte
	# 	return steer

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
			if self.satellite is None or self.telem is None or self.heading is None:
				return

			msg = Twist()

			res = self.get_goal_motion()				

			if not res.success:
				print(res.message)
				return
			
			v, head = res.x

			print(f"calculated targer v {v} and heading {head}")

			# get the motion of the ego vehicle
			v_ego = np.sqrt(self.telem.twist.twist.linear.x**2 + self.telem.twist.twist.linear.y**2)
			head_ego = self.heading.data
			
			vel_accel = self.velocity_controller(v, v_ego)
			delta = self.heading_controller(head_ego, v_ego, head)
			new_speed = v_ego + vel_accel*self.broadcast_interval
			new_speed = min(new_speed, SPEED_LIMIT)

			if self.iteration_error_count == self.car:
				new_speed = 0

			print(f"setting speed {new_speed} m/s and heading delta {delta} | Current speed and heading: {v_ego} {head_ego}\n")

			delta = np.radians(delta)
			head_ego = np.radians(head_ego)

			msg.linear.x = -new_speed * math.sin(delta)
			msg.linear.y = new_speed * math.cos(delta)
			msg.linear.z = 0.0
			msg.angular.x = 0.0
			msg.angular.y = 0.0
			msg.angular.z = 0.0

			print(msg.linear.x, msg.linear.y, msg.angular.z)

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
			print("MISSION COMPLETE")
			rclpy.shutdown()
		
		# if the mission is aborted, turn off all motors immediately.
		if self.mission_status == ABORT:
			print("ABORTING")

			now = datetime.now()
			formatted_date = now.strftime('%H_%M_%d')
			
			with open(f"/home/nvidia/ros2_ws/src/cavrel-platooning/missions/datapoints/py3_{self.car}_{self.track_name}_{formatted_date}.pkl", "wb") as f:
				pickle.dump(self.datapoints, f)
			self.datapoints = None

			# with open(f"/home/nvidia/ros2_ws/src/cavrel-platooning/missions/datapoints/py3_{self.car}_{self.track_name}_{formatted_date}_LINE.pkl", "wb") as f:
			# 	pickle.dump(self.lines, f)
			# self.lines = None
			
			# with open(f"/home/nvidia/ros2_ws/src/cavrel-platooning/missions/datapoints/py3_{self.car}_{self.track_name}_{formatted_date}_CLOSEST.pkl", "wb") as f:
			# 	pickle.dump(self.closest, f)
			# self.closest = None

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
	args = parser.parse_args()
	with open(args.track_path, 'r') as f:
		track = json.load(f)

	car_number = args.car_number
	broadcast_interval = args.broadcast_int
	drop_rate = args.drop_rate
	track_name = track['name']
	center_lat = track['center']['lat']
	center_lon = track['center']['lon']
	center_orientation = DUE_EAST

	rclpy.init(args=None)
	udp_publisher = UDPPublisher(car_number, broadcast_interval, drop_rate, center_lat, center_lon, center_orientation, track_name)
	executor = MultiThreadedExecutor()
	executor.add_node(udp_publisher)
	executor.spin()