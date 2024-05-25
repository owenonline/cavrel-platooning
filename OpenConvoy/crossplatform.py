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
    def __init__(self, car_number, car_length, broadcast_interval, listen_interval, drop_rate, track_name,
                 center_lat, center_lon, center_orientation, save_path, speed_limit, kpv=None, kdv=None, k=None, ks=None, 
                 follow_distance=None, **kwargs):
        self.car_number = car_number
        self.car_length = car_length
        self.broadcast_interval = broadcast_interval
        self.listen_interval = listen_interval
        self.drop_rate = drop_rate
        self.track_name = track_name
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.center_orientation = center_orientation
        self.save_path = save_path
        self.kpv = kpv
        self.kdv = kdv
        self.k = k
        self.ks = ks
        self.follow_distance = follow_distance
        self.speed_limit = speed_limit
        
        for key, value in kwargs.items():
            setattr(self, key, value)

class BasicSafetyMessage(object):
    """Implements the J2735 Basic Safety Message (BSM) data structure."""

    def __init__(self, time=None, latitude=None, longitude=None, elevation=None, position_accuracy=None,
                 speed=None, heading=None, acceleration=None, yaw_rate=None, steering_wheel_angle=None,
                 transmission_state=None, brake_system_status=None, vehicle_length=None, vehicle_width=None,
                 path_history=None, path_prediction=None, exterior_lights=None, event_flags=None):
        self.time = time
        self.latitude = latitude
        self.longitude = longitude
        self.elevation = elevation
        self.position_accuracy = position_accuracy
        self.speed = speed
        self.heading = heading
        self.acceleration = acceleration
        self.yaw_rate = yaw_rate
        self.steering_wheel_angle = steering_wheel_angle
        self.transmission_state = transmission_state
        self.brake_system_status = brake_system_status
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.path_history = path_history if path_history is not None else []
        self.path_prediction = path_prediction if path_prediction is not None else []
        self.exterior_lights = exterior_lights if exterior_lights is not None else []
        self.event_flags = event_flags if event_flags is not None else {}

    def to_json(self):
        return json.dumps(self.__dict__)

class Control(object):
    def __init__(self, rosargs, *args):
        self.args = rosargs
        super(Control, self).__init__(*args)

        self.datapoints = []
        self.beacon_started = False
        self.telem = None
        self.satellite = None
        self.heading = None
        self.imu = None
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
    
    def should_broadcast(self):
        """Determines whether the car should broadcast a message. If you want to implement any kind of event triggered communication, override this function in your subclass to do so."""
        return True

    def should_listen(self, sender_number):
        """Determines whether the car should listen to the broadcast from the sender based on the car's position in the platoon.
        Override this function in your subclass to implement a different network topology, or leave as-is for APLF."""
        return sender_number < self.args.car_number

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
    
    def minimization_objective(self, params):
        """Cost function to minimize to implement cooperative driving policy. By default implements platooning. Override this function in your subclass to implement a different policy."""

        # v, head = params
        v = params[0]
        head = 0
        head = np.radians(head)
        x, y = self.coords_to_local(self.state.latitude, self.state.longitude)

        total_cost = 0
        for _, value in self.car_positions.items():
            value = value[-1]
            x_target, y_target = self.coords_to_local(value.latitude, value.longitude)
            position = self.args.car_number - value.event_flags['car']
            head_target, v_target = np.radians(value.heading), value.speed

            goal_follow_distance = self.args.follow_distance*position + self.args.car_length*(position - 1)

            x_sim_target = x_target + v_target*np.sin(head_target)*self.args.broadcast_interval
            y_sim_target = y_target + v_target*np.cos(head_target)*self.args.broadcast_interval
            x_sim_ego = x + v*np.sin(head)*self.args.broadcast_interval
            y_sim_ego = y + v*np.cos(head)*self.args.broadcast_interval

            x_goal = x_sim_target - goal_follow_distance*np.sin(head_target)
            y_goal = y_sim_target - goal_follow_distance*np.cos(head_target)

            # print("goal follow: {gfd}, head target: {th}, vtarget: {vt}, selected {v} {h}".format(gfd=goal_follow_distance, th=value.heading, vt=value.speed, v=v, h=np.rad2deg(head)))

            total_cost += np.sqrt((x_goal - x_sim_ego)**2 + (y_goal - y_sim_ego)**2)

        return total_cost
    
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
        for key, value in self.car_positions.items():
            m1, m2 = value[-2:]
            x1, y1 = self.coords_to_local(m1.latitude, m1.longitude)
            x2, y2 = self.coords_to_local(m2.latitude, m2.longitude)
            points.append((x1, y1))
            points.append((x2, y2))

            if key == self.args.car_number - 1: # we use the car closest to the ego vehicle as the initial guess for the line to ensure convergence
                initial_guess = [x1, y1, x2 - x1, y2 - y1]

        # get the local coordinates of the ego car
        lat1, lon1 = self.state.latitude, self.state.longitude
        ex1, ey1 = self.coords_to_local(lat1, lon1)

        def distances_to_line(params, points):
            x0, y0, dx, dy = params
            distances = []
            for (x, y) in points:
                distance = self.distance_to_line(x0, y0, dx, dy, x, y)
                distances.append(distance)
            return distances
        
        # compute a line of best fit using the last 2 positions of all other cars
        # this allows us to accurately calculate the cross track error between where we are and where we should be to be most directly in line with the other vehicles
        # NOTE: this will start to break as you get more cars going around a curve, at which point we should switch to a curved line of best fit
        result = least_squares(distances_to_line, initial_guess, args=(points,))

        x0_opt, y0_opt, dx_opt, dy_opt = result.x
        heading_diff = target_head - head_ego

        # if heading_diff > 180:
        #     heading_diff -= 360
        # elif heading_diff < -180:
        #     heading_diff += 360

        dist = self.distance_to_line(x0_opt, y0_opt, dx_opt, dy_opt, ex1, ey1)
        cte = np.arctan2(self.args.k*dist, self.args.ks + v_ego)
        cte = np.rad2deg(cte)

        print("heading error: {heading_diff} crosstrack error: {cte} heading: {egohead} ego pos: ({ex1}, {ey1})".format(heading_diff=heading_diff, cte=cte, egohead=head_ego, ex1=ex1, ey1=ey1))

        steer = heading_diff# + cte
        return steer
    
    @property
    def sensors_ok(self):
        return self.satellite is not None and self.telem is not None and self.heading is not None and self.imu is not None
    
    @property
    def state(self):
        return BasicSafetyMessage(
            time=time(),
            latitude=self.satellite.latitude,
            longitude=self.satellite.longitude,
            elevation=self.satellite.altitude,
            speed=np.sqrt(self.telem.twist.twist.linear.x**2 + self.telem.twist.twist.linear.y**2),
            heading=self.heading.data,
            acceleration=np.sqrt(self.imu.linear_acceleration.x**2 + self.imu.linear_acceleration.y**2),
            yaw_rate=self.imu.angular_velocity.z,
            event_flags={
                "abort": self.mission_status in [ABORT, DISARMING, MISSIONCOMPLETE],
                "car": self.args.car_number,
            }
        )
    
    def _listen(self):
        """Listens for and stores the latest broadcasts from other cars in the network."""

        data, _ = self.listen_sock.recvfrom(1024)
        data_json = json.loads(data.decode())
        bsm_message = BasicSafetyMessage(**data_json)

        if bsm_message.event_flags['abort'] and not self.mission_status in [ABORT, DISARMING, MISSIONCOMPLETE]:
            self._disarm()
        
        if self.should_listen(bsm_message.event_flags['car']):
            self.car_positions[bsm_message.event_flags['car']].append(bsm_message)
            self.car_positions[bsm_message.event_flags['car']] = self.car_positions[bsm_message.event_flags['car']][-4:]

        # determine that the beacon is broadcasting if we have a history of 2 messages from it
        if len(self.car_positions[0]) >= 2:
            self.beacon_started = True

    def _broadcast(self):
        """Broadcasts the car's current GPS position and heading to all other cars in the network, dropping packets at a specified rate."""

        if self.satellite is None or self.heading is None or (self.args.drop_rate > 0 and random.random() < self.args.drop_rate):
            return
        
        if self.should_broadcast():
            msg = self.state.to_json()
            msg = msg.encode()
            self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

    def _get_goal_motion(self):
        """Calculates the target speed and heading for the ego vehicle based on the positions of the other cars in the network. 
        Uses the implemented minimization objective, which by default is for platooning."""

        # bounds = [(0, 10), (-360, 360)]
        bounds = [(0, 2.5)]
        head, v = self.car_positions[0][-1].heading, self.car_positions[0][-1].speed
        # res = minimize(lambda params: self.minimization_objective(params), [v, head], method='SLSQP', bounds=bounds)
        res = minimize(lambda params: self.minimization_objective(params), [v], method='SLSQP', bounds=bounds)
        return res

    def _get_applied_motion(self, v, head):
        """Given goal velocity and heading, uses the velocity and heading controllers to calculate a smooth change in x and y velocity to send to the autopilot."""

        v_ego = self.state.speed
        head_ego = self.state.heading

        vel_accel = self.velocity_controller(v, v_ego)
        new_speed = v_ego + vel_accel*self.args.broadcast_interval
        new_speed = min(new_speed, self.args.speed_limit)

        delta = self.heading_controller(head_ego, v_ego, head)
        delta = np.clip(delta, -30, 30)
        delta_rad = np.radians(delta)

        x = -new_speed * math.sin(delta_rad)
        y = new_speed * math.cos(delta_rad)

        print("applying delta {delta} and speed {speed} {x} {y}".format(delta=delta, speed=new_speed, x=x, y=y))

        return x, y

    def _disarm(self):
        """implemented in ros-specific child class"""
        pass

    def _telem_listener_callback(self, msg):
        """Saves the latest telemetry message"""
        self.telem = msg

    def _satellite_listener_callback(self, msg):
        """Saves the latest GPS message"""
        self.satellite = msg

    def _heading_listener_callback(self, msg):
        """Saves the latest heading message"""
        self.heading = msg

    def _accel_listener_callback(self, msg):
        """Saves the latest acceleration message"""
        self.imu = msg

    def _update_datapoints(self):
        to_add = []

        for _, value in self.car_positions.items():
            value = value[-1]
            x, y = self.coords_to_local(value.latitude, value.longitude)
            to_add.append((x, y, value.heading, value.speed, value.acceleration, time()))

        ex, ey = self.coords_to_local(self.state.latitude, self.state.longitude)
        to_add.append((ex, ey, self.state.heading, self.state.speed, self.state.acceleration, time()))

        self.datapoints.append(to_add)

    def _save_data(self):
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
