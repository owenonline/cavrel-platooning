import random
import rospy
import struct
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
import socket
from time import sleep, time
from datetime import datetime
import json
import threading
import numpy as np
import math
import pickle
from collections import defaultdict
from scipy.optimize import minimize, minimize_scalar, least_squares
import pyproj
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
CAR_LENGTH = 0.779
FOLLOW_DISTANCE = 2.0
DUE_EAST = 90
SPEED_LIMIT = 5
geodesic = pyproj.Geod(ellps='WGS84')

# set up args
parser = argparse.ArgumentParser()
parser.add_argument('--track_path', type=str, default="missions/tracks/bus_loop_large.json")
parser.add_argument('--broadcast_int', type=float, default=0.1)
parser.add_argument('--drop_rate', type=float, default=0.0)
parser.add_argument('--car_number', type=int, default=2)

class UDPPublisher:
    def __init__(self, car_number, broadcast_interval, drop_rate, center_lat, center_lon, center_orientation, track_name):
        rospy.init_node('udp_publisher')

        # add args to object
        self.car = car_number
        self.broadcast_interval = broadcast_interval
        self.drop_rate = drop_rate
        self.center_latitude = center_lat
        self.center_longitude = center_lon
        self.center_orientation = center_orientation
        self.track_name = track_name

        # set starting vars
        self.datapoints = []
        self.iteration_error_count = 0
        self.mission_status = MISSIONSTART
        self.telem = None
        self.satellite = None
        self.heading = None
        self.accel = None
        self.car_positions = defaultdict(list)
        self.movement_message = Twist()

        # socket broadcast setup
        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        ttl = struct.pack('b', 1)
        self.broadcast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', 5004))
        group = socket.inet_aton('224.0.0.1')
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        # sensor subscription setup
        self.telem_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.telem_listener_callback)
        self.satellite_subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.satellite_listener_callback)
        self.heading_subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.heading_listener_callback)
        self.accel_subscriber = rospy.Subscriber('/mavros/imu/data', Imu, self.accel_listener_callback)

        # publisher setup
        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=20)

        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/command')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.rate = rospy.Rate(20)
        self.move_thread = threading.Thread(target=self.movement_callback)
        self.move_thread.daemon = True
        self.move_thread.start()

        # set up killswitch
        self.killswitch_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        self.stop_thread = threading.Thread(target=self.listen_for_stop)
        self.stop_thread.daemon = True
        self.stop_thread.start()

        rospy.Timer(rospy.Duration(self.broadcast_interval), self.broadcast_timer_callback)
        rospy.Timer(rospy.Duration(LISTEN_INTERVAL), self.listen_timer_callback)
        rospy.Timer(rospy.Duration(self.broadcast_interval), self.mission_timer_callback)

    def listen_for_stop(self):
        """Kills the mission if the user presses ENTER."""
        
        while True:
            raw_input()
            now = datetime.now()
            formatted_date = now.strftime('%H_%M_%d')
            file_path = "missions/datapoints/py2_{car}_{track}_{date}.pkl".format(car=self.car, track=self.track_name, date=formatted_date)
            
            with open(file_path, "wb") as f:
                pickle.dump(self.datapoints, f)
            self.mission_status = DISARMING
            response = self.arming_service(False)
            if response.success:
                print("Disarmed")
                self.mission_status = MISSIONCOMPLETE

    def broadcast_timer_callback(self, event):
        """Broadcasts the car's current GPS position and heading to all other cars in the network, dropping packets at a specified rate."""

        if self.satellite is None or self.heading is None or (self.drop_rate > 0 and random.random() < self.drop_rate):
            return

        msg = json.dumps({"head": self.heading.data, "car": self.car, "lat": self.satellite.latitude, "lon": self.satellite.longitude, "time": time(), "abort": self.mission_status == ABORT, "accel": (self.accel.x, self.accel.y)})
        msg = msg.encode()
        self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

    def movement_callback(self):
        """Publishes movement commands at the rate required to keep the vehicle armed"""

        while True:
            self.publisher.publish(self.movement_message)
            self.rate.sleep()

    def listen_timer_callback(self, event):
        """Listens for and stores the latest broadcasts from other cars in the network."""

        data, _ = self.listen_sock.recvfrom(1024)
        data_json = json.loads(data.decode())
        if data_json['abort']:
            self.mission_status = ABORT
        if data_json['car'] <= self.car:
            position_update = (data_json['lat'], data_json['lon'], data_json['head'], data_json['time'], data_json['accel'])
            self.car_positions[data_json['car']].append(position_update)
            self.car_positions[data_json['car']] = self.car_positions[data_json['car']][-4:]

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
            # if no points received, say that the vehicle is exactly where the ego vehicle is and not moving
            # so the optimizded doesn't do anything either
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

        self.datapoints.append([(x, y, h, v, a[0], a[1]) for x, y, h, v, _, a in targets] + [(ex, ey, eh, ev, self.accel.x, self.accel.y)])

        def minimization_objective(params):
            v, head = params
            head = np.radians(head)
            x, y = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)

            total_cost = 0
            for target in targets:
                x_target, y_target, head_target, v_target, position, _ = target
                head_target = np.radians(head_target)
                goal_follow_distance = FOLLOW_DISTANCE*position + CAR_LENGTH*(position - 1)

                x_sim_target = x_target + v_target*np.sin(head_target)*self.broadcast_interval
                y_sim_target = y_target + v_target*np.cos(head_target)*self.broadcast_interval
                x_sim_ego = x + v*np.sin(head)*self.broadcast_interval
                y_sim_ego = y + v*np.cos(head)*self.broadcast_interval

                x_goal = x_sim_target - goal_follow_distance*np.sin(head_target)
                y_goal = y_sim_target - goal_follow_distance*np.cos(head_target)

                total_cost += np.sqrt((x_goal - x_sim_ego)**2 + (y_goal - y_sim_ego)**2)

            return total_cost

        bounds = [(0, 10), (-360, 360)]
        _, _, head, v, _, _ = targets[0]
        guesses = [[0, head], [v, head], [5, head]]
        best_score = np.inf
        best = None
        for guess in guesses:
            res = minimize(minimization_objective, guess, method='SLSQP', bounds=bounds)
            if res.fun < best_score:
                best_score = res.fun
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
        for i in range(self.car):
            if len(self.car_positions[i]) < 2:
                continue

            (lat1, lon1, _, _, _), (lat2, lon2, _, _, _)= self.car_positions[i][-2:]
            x1, y1 = self.coords_to_local(lat1, lon1)
            x2, y2 = self.coords_to_local(lat2, lon2)
            points.append((x1, y1))
            points.append((x2, y2))

            if i == self.car - 1:
                initial_guess = [x2, y2, x2 - x1, y2 - y1]

        # if no points received, steer is 0
        if len(points) == 0 or initial_guess is None:
            return 0

        lat1, lon1 = self.satellite.latitude, self.satellite.longitude
        ex1, ey1 = self.coords_to_local(lat1, lon1)

        def distances_to_line(params, points):
            x0, y0, dx, dy = params
            distances = []
            for (x, y) in points:
                distance, _ = self.distance_to_line(x0, y0, dx, dy, x, y)
                distances.append(distance)
            return distances

        result = least_squares(distances_to_line, initial_guess, args=(points,))
        x0_opt, y0_opt, dx_opt, dy_opt = result.x
        heading_diff = target_head - head_ego

        dist, closest = self.distance_to_line(x0_opt, y0_opt, dx_opt, dy_opt, ex1, ey1)
        v_ego = max(v_ego, 1)
        cte = np.arctan2(K*dist, v_ego)
        cte = np.rad2deg(cte)
        steer = heading_diff + cte
        return steer

    def mission_timer_callback(self, event):
        """Main loop for vehicle control. Handles the arming, moving, and disarming of the rover."""

        if self.mission_status == MISSIONSTART:
            print("Switching to offboard mode.")
            try:
                response = self.set_mode_service(0, 'OFFBOARD')
                if response.mode_sent:
                    print("arming")
                    self.mission_status = ARMING
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
        elif self.mission_status == ARMING:
            try:
                sleep(4)
                response = self.arming_service(True)
                if response.success:
                    print("Moving")
                    self.mission_status = MOVING
                else:
                    print(response)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
        elif self.mission_status == MOVING:
            print("Moving")
            res = self.get_goal_motion()
            if not res.success:
                print("Optimization failed:", res.message)
                return
            v, head = res.x
            v_ego = np.sqrt(self.telem.twist.twist.linear.x**2 + self.telem.twist.twist.linear.y**2)
            head_ego = self.heading.data
            vel_accel = self.velocity_controller(v, v_ego)
            delta = self.heading_controller(head_ego, v_ego, head)
            new_speed = v_ego + vel_accel * self.broadcast_interval
            new_speed = min(new_speed, SPEED_LIMIT)
            delta_rad = np.radians(delta)
            msg = Twist()
            if not self.iteration_error_count == self.car:
                msg.linear.x = -new_speed * math.sin(delta_rad)
                msg.linear.y = new_speed * math.cos(delta_rad)
            # self.publisher.publish(msg)
            print(msg.linear.x, msg.linear.y)
            self.movement_message = msg
        elif self.mission_status == DISARMING:
            msg = Twist()
            self.publisher.publish(msg)
        elif self.mission_status == MISSIONCOMPLETE:
            rospy.signal_shutdown("Mission completed successfully.")
        elif self.mission_status == ABORT:
            print("Aborting mission.")
            now = datetime.now()
            formatted_date = now.strftime('%H_%M_%d')
            file_path = "missions/datapoints/py2_{car}_{track}_{date}.pkl".format(car=self.car, track=self.track_name, date=formatted_date)
            
            with open(file_path, "wb") as f:
                pickle.dump(self.datapoints, f)

            self.datapoints = None
            self.killswitch_service(False, 400, 0, 0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0)
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

    udp_publisher = UDPPublisher(car_number, broadcast_interval, drop_rate, center_lat, center_lon, center_orientation, track_name)
    rospy.spin()