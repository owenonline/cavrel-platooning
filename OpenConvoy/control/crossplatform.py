from collections import defaultdict
import socket
import struct
import json
import random
from time import time
import numpy as np
import math
from scipy.optimize import minimize, least_squares
from datetime import datetime
import pickle
import os

MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4
ABORT = -1

EARTH_RADIUS = 6371e3
DUE_EAST = 90

class ROSArgs:
    def __init__(self, args):
        self.car_number = int(args.car_number)
        self.car_length = float(args.car_length)
        self.broadcast_interval = float(args.broadcast_interval)
        self.listen_interval = float(args.listen_interval)
        self.drop_rate = float(args.drop_rate)
        self.track_name = args.track_name
        self.center_lat = float(args.center_lat)
        self.center_lon = float(args.center_lon)
        self.center_orientation = float(args.center_orientation)
        self.save_path = args.save_path
        self.kpv = float(args.kpv)
        self.kdv = float(args.kdv)
        self.k = float(args.k)
        self.follow_distance = float(args.follow_distance)
        self.speed_limit = float(args.speed_limit)

class Control(object):
    def __init__(self, rosargs, *args):
        self.args = rosargs
        super(Control, self).__init__(*args)

        self.datapoints = []
        self.beacon_started = False
        self.telem = None
        self.satellite = None
        self.heading = None
        self.accel = None
        self.car_positions = defaultdict(list)
        self.mission_status = MISSIONSTART

        # set up UDP communication
        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        ttl = struct.pack('b', 1)
        self.broadcast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', 5004))
        group = socket.inet_aton('224.0.0.1')
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def broadcast_timer_callback(self, event):
        """Broadcasts the car's current GPS position and heading to all other cars in the network, dropping packets at a specified rate."""

        if self.satellite is None or self.heading is None or (self.args.drop_rate > 0 and random.random() < self.args.drop_rate):
            return

        msg = json.dumps({"head": self.heading.data, "car": self.args.car_number, "lat": self.satellite.latitude, "lon": self.satellite.longitude, "time": time(), "abort": self.mission_status in [ABORT, DISARMING, MISSIONCOMPLETE], "accel": (self.accel.x, self.accel.y)})
        msg = msg.encode()
        self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

    def listen_timer_callback(self, event):
        """Listens for and stores the latest broadcasts from other cars in the network."""

        data, _ = self.listen_sock.recvfrom(1024)
        data_json = json.loads(data.decode())

        if data_json['abort']:
            self.disarm()
        
        if data_json['car'] <= self.args.car_number:
            position_update = (data_json['lat'], data_json['lon'], data_json['head'], data_json['time'], data_json['accel'])
            self.car_positions[data_json['car']].append(position_update)
            self.car_positions[data_json['car']] = self.car_positions[data_json['car']][-4:]

        # determine that the beacon is broadcasting if we have a history of 2 messages from it
        if len(self.car_positions[0]) >= 2:
            self.beacon_started = True

    def disarm(self):
        """implemented in child class"""
        pass

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

    @property
    def sensors_ok(self):
        return self.satellite is not None and self.telem is not None and self.heading is not None and self.accel is not None

    def coords_to_local(self, target_lat, target_lon):
        """Converts GPS coordinates to local cartesian coordinates with respect to the track center point."""
        
        target_lat_rad, target_lon_rad = math.radians(target_lat), math.radians(target_lon)
        current_lat_rad, current_lon_rad = math.radians(self.args.center_lat), math.radians(self.args.center_lon)

        x = EARTH_RADIUS * (target_lon_rad - current_lon_rad) * math.cos((current_lat_rad + target_lat_rad) / 2)
        y = EARTH_RADIUS * (target_lat_rad - current_lat_rad)

        angle = math.radians(self.args.center_orientation - DUE_EAST)
        qx = math.cos(angle) * x - math.sin(angle) * y
        qy = math.sin(angle) * x + math.cos(angle) * y
        return qx, qy

    def get_goal_motion(self):
        """Calculates the target speed and heading for the ego vehicle based on the positions of the other cars in the network."""

        ex, ey = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)
        eh = self.heading.data
        ev = np.sqrt(self.telem.twist.twist.linear.x**2 + self.telem.twist.twist.linear.y**2)

        targets = []
        self.iteration_error_count = 0
        for i in range(self.args.car_number):
            # if no points received, say that the vehicle is exactly where the ego vehicle is and not moving
            # so the optimizded doesn't do anything either
            if len(self.car_positions[i]) < 2:
                targets.append((ex, ey, eh, 0, self.args.car_number - i, (0,0)))
                self.iteration_error_count += 1
            else:
                (lat1, lon1, head1, time1, _), (lat2, lon2, head2, time2, accel) = self.car_positions[i][-2:]

                x1, y1 = self.coords_to_local(lat1, lon1)
                x2, y2 = self.coords_to_local(lat2, lon2)

                velocity = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)/(time2 - time1)
                heading = head2
                targets.append((x2, y2, heading, velocity, self.args.car_number - i, accel))

        self.datapoints.append([(x, y, h, v, a[0], a[1], time()) for x, y, h, v, _, a in targets] + [(ex, ey, eh, ev, self.accel.x, self.accel.y, time())])

        def minimization_objective(params):
            v, head = params
            head = np.radians(head)
            x, y = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)

            total_cost = abs(v - ev)
            for target in targets:
                x_target, y_target, head_target, v_target, position, _ = target
                head_target = np.radians(head_target)
                goal_follow_distance = self.args.follow_distance*position + self.args.car_length*(position - 1)

                x_sim_target = x_target + v_target*np.sin(head_target)*self.args.broadcast_interval
                y_sim_target = y_target + v_target*np.cos(head_target)*self.args.broadcast_interval
                x_sim_ego = x + v*np.sin(head)*self.args.broadcast_interval
                y_sim_ego = y + v*np.cos(head)*self.args.broadcast_interval

                x_goal = x_sim_target - goal_follow_distance*np.sin(head_target)
                y_goal = y_sim_target - goal_follow_distance*np.cos(head_target)

                total_cost += np.sqrt((x_goal - x_sim_ego)**2 + (y_goal - y_sim_ego)**2)

            return total_cost

        bounds = [(0, 10), (-360, 360)]
        _, _, head, v, _, _ = targets[0]
        res = minimize(minimization_objective, [v, head], method='SLSQP', bounds=bounds)
        return res
    
    def velocity_controller(self, v, v_ego):
        """PD controller for velocity control. Computes the acceleration needed to match the target speed."""
        accel = self.args.kpv*(v - v_ego) + self.args.kdv*(v - v_ego)/self.args.broadcast_interval
        return accel
    
    def distance_to_line(self, x0, y0, dx, dy, x, y):
        """Computes the distance between a point and a line."""
        lambda_val = ((x - x0) * dx + (y - y0) * dy) / (dx**2 + dy**2)
        closest_point = np.array([x0 + lambda_val * dx, y0 + lambda_val * dy])
        distance = np.linalg.norm(closest_point - np.array([x, y]))
        distance = np.nan_to_num(distance)
        return distance
    
    def heading_controller(self, head_ego, v_ego, target_head):
        """Stanley controller for heading control. Computes the cross track error and heading difference between the ego car and the target car.
        CURRENTLY APPROXIMATES CROSS-TRACK ERROR WITH A STRAIGHT LINE FIT."""
        points = []
        initial_guess = None

        # get the local coordinates of the last two locations of all other cars
        for i in range(self.args.car_number):
            if len(self.car_positions[i]) < 2:
                continue

            (lat1, lon1, _, _, _), (lat2, lon2, _, _, _) = self.car_positions[i][-2:]
            x1, y1 = self.coords_to_local(lat1, lon1)
            x2, y2 = self.coords_to_local(lat2, lon2)
            points.append((x1, y1))
            points.append((x2, y2))

            if i == self.args.car_number - 1: # we use the car closest to the ego vehicle as the initial guess for the line to ensure convergence
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

        if heading_diff > 180:
            heading_diff -= 360
        elif heading_diff < -180:
            heading_diff += 360

        dist = self.distance_to_line(x0_opt, y0_opt, dx_opt, dy_opt, ex1, ey1)
        v_ego = max(v_ego, 1)
        cte_scale = min(1, v_ego/1) # reduces the influences of CTE when speed is low, to prevent oversteering
        cte = np.arctan2(self.args.k*dist, v_ego)
        cte = np.rad2deg(cte) * cte_scale

        print("heading error: {heading_diff} crosstrack error: {cte} line params: {result.x} ego pos: ({ex1}, {ey1})".format(heading_diff=heading_diff, cte=cte, result=result, ex1=ex1, ey1=ey1))

        steer = heading_diff + cte
        return steer
    
    def get_applied_motion(self, v, head):
        """Given goal velocity and heading, uses a PD controller for velocity and a Stanley controller for heading to calculate a smooth change in x and y velocity to send to the autopilot."""
        
        v_ego = np.sqrt(self.telem.twist.twist.linear.x**2 + self.telem.twist.twist.linear.y**2)
        head_ego = self.heading.data

        vel_accel = self.velocity_controller(v, v_ego)
        new_speed = v_ego + vel_accel*self.args.broadcast_interval
        new_speed = min(new_speed, self.args.speed_limit)

        delta = self.heading_controller(head_ego, v_ego, head)
        delta = np.clip(delta, -30, 30)
        delta_rad = np.radians(delta)

        x = -new_speed * math.sin(delta_rad)
        y = new_speed * math.cos(delta_rad)

        return x, y

    def save_data(self):
        now = datetime.now()
        formatted_date = now.strftime('%m_%d')
        formatted_time = now.strftime('%H_%M')
        save_path = os.path.join(self.args.save_path, self.args.track_name, formatted_date)

        if not os.path.exists(save_path):
            os.makedirs(save_path)
        
        file_name = "py2_{car}_{time}_{droprate}.pkl".format(car=self.args.car_number, time=formatted_time, droprate=self.args.drop_rate)
        file_path = os.path.join(save_path, file_name)
        
        with open(file_path, "wb") as f:
            pickle.dump(self.datapoints, f)