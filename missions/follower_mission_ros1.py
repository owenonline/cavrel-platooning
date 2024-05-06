import rospy
import struct
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
import socket
from time import sleep, time
import json
import threading
import numpy as np
import math
import pickle
from collections import defaultdict
from scipy.optimize import minimize, least_squares
import pyproj

MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4
ABORT = -1

NUM_CARS = 2  # number of total cars in the network, including the ego vehicle
EARTH_RADIUS = 6371e3  # earth radius in meters
KPV = 0.3
KDV = 0.5
K = 0.3
BROADCAST_INTERVAL = 0.1  # same for all cars
LISTEN_INTERVAL = 0.01
MAX_STEER = 30
CAR_LENGTH = 0.779
FOLLOW_DISTANCE = 2.0  # meters behind the immediate preceding vehicle, 4 meters behind the second preceding vehicle, etc.
DUE_EAST = 90
SPEED_LIMIT = 3.0
geodesic = pyproj.Geod(ellps='WGS84')
# center_latitude = (28.607980 + 28.607292) / 2
# center_longitude = (-81.195662 + -81.194750) / 2
center_latitude = 28.602202442735443 
center_longitude = -81.19671976053279
center_orientation = DUE_EAST

class UDPPublisher:
    def __init__(self, car):
        rospy.init_node('udp_publisher')
        self.car = car
        self.datapoints = []
        self.mission_status = MISSIONSTART
        self.telem = None
        self.satellite = None
        self.heading = None
        self.car_positions = defaultdict(list)

        self.movement_message = Twist()

        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        ttl = struct.pack('b', 1)
        self.broadcast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', 5004))
        group = socket.inet_aton('224.0.0.1')
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.telem_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.telem_listener_callback)
        self.satellite_subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.satellite_listener_callback)
        self.heading_subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.heading_listener_callback)

        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=20)

        rospy.Timer(rospy.Duration(BROADCAST_INTERVAL), self.broadcast_timer_callback)
        rospy.Timer(rospy.Duration(LISTEN_INTERVAL), self.listen_timer_callback)
        rospy.Timer(rospy.Duration(BROADCAST_INTERVAL), self.mission_timer_callback)

        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/command')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.killswitch_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        self.stop_thread = threading.Thread(target=self.listen_for_stop)
        self.stop_thread.daemon = True
        self.stop_thread.start()

        self.rate = rospy.Rate(20)
        self.move_thread = threading.Thread(target=self.movement_callback)
        self.move_thread.daemon = True
        self.move_thread.start()

    def listen_for_stop(self):
        while True:
            command = input()
            if command.upper() == 'K':
                self.mission_status = ABORT

    def broadcast_timer_callback(self, event):
        if self.satellite is None:
            return

        msg = json.dumps({"head": self.heading.data, "car": self.car, "lat": self.satellite.latitude, "lon": self.satellite.longitude, "time": time(), "abort": self.mission_status == ABORT})
        msg = msg.encode()
        self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))

    def movement_callback(self):
        while True:
            self.publisher.publish(self.movement_message)
            self.rate.sleep()

    def listen_timer_callback(self, event):
        data, _ = self.listen_sock.recvfrom(1024)
        data_json = json.loads(data.decode())
        if data_json['abort']:
            self.mission_status = ABORT
        if data_json['car'] <= self.car:
            position_update = (data_json['lat'], data_json['lon'], data_json['head'], data_json['time'])
            self.car_positions[data_json['car']].append(position_update)
            self.car_positions[data_json['car']] = self.car_positions[data_json['car']][-4:]

    def telem_listener_callback(self, msg):
        self.telem = msg

    def satellite_listener_callback(self, msg):
        self.satellite = msg

    def heading_listener_callback(self, msg):
        self.heading = msg

    def get_goal_motion(self):
        targets = []
        for i in range(self.car):
            (lat1, lon1, head1, time1), (lat2, lon2, head2, time2) = self.car_positions[i][-2:]

            x1, y1 = self.coords_to_local(lat1, lon1)
            x2, y2 = self.coords_to_local(lat2, lon2)

            velocity = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)/(time2 - time1)
            heading = head2
            targets.append((x2, y2, heading, velocity, self.car - i))

        ex, ey = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)
        eh = self.heading.data
        ev = np.sqrt(self.telem.twist.twist.linear.x**2 + self.telem.twist.twist.linear.y**2)
        self.datapoints.append([(x, y, h, v) for x, y, h, v, _ in targets] + [(ex, ey, eh, ev)])

        def minimization_objective(params):
            v, head = params
            head = np.radians(head)
            x, y = self.coords_to_local(self.satellite.latitude, self.satellite.longitude)

            total_cost = 0
            for target in targets:
                x_target, y_target, head_target, v_target, position = target
                head_target = np.radians(head_target)
                goal_follow_distance = FOLLOW_DISTANCE*position + CAR_LENGTH*(position - 1)

                x_sim_target = x_target + v_target*np.sin(head_target)*BROADCAST_INTERVAL
                y_sim_target = y_target + v_target*np.cos(head_target)*BROADCAST_INTERVAL
                x_sim_ego = x + v*np.sin(head)*BROADCAST_INTERVAL
                y_sim_ego = y + v*np.cos(head)*BROADCAST_INTERVAL

                x_goal = x_sim_target - goal_follow_distance*np.sin(head_target)
                y_goal = y_sim_target - goal_follow_distance*np.cos(head_target)

                total_cost += np.sqrt((x_goal - x_sim_ego)**2 + (y_goal - y_sim_ego)**2)

            return total_cost

        bounds = [(0, -360), (10, 360)]
        _, _, head, v, _ = targets[0]
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
        target_lat_rad, target_lon_rad = math.radians(target_lat), math.radians(target_lon)
        current_lat_rad, current_lon_rad = math.radians(center_latitude), math.radians(center_longitude)

        x = EARTH_RADIUS * (target_lon_rad - current_lon_rad) * math.cos((current_lat_rad + target_lat_rad) / 2)
        y = EARTH_RADIUS * (target_lat_rad - current_lat_rad)

        angle = math.radians(center_orientation - DUE_EAST)
        qx = math.cos(angle) * x - math.sin(angle) * y
        qy = math.sin(angle) * x + math.cos(angle) * y
        return qx, qy

    def velocity_controller(self, v, v_ego):
        accel = KPV*(v - v_ego) + KDV*(v - v_ego)/BROADCAST_INTERVAL
        return accel

    def distance_to_line(self, x0, y0, dx, dy, x, y):
        lambda_val = ((x - x0) * dx + (y - y0) * dy) / (dx**2 + dy**2)
        closest_point = np.array([x0 + lambda_val * dx, y0 + lambda_val * dy])
        distance = np.linalg.norm(closest_point - np.array([x, y]))
        distance = np.nan_to_num(distance)
        return distance, closest_point

    def heading_controller(self, head_ego, v_ego, target_head):
        points = []
        initial_guess = None
        for i in range(self.car):
            (lat1, lon1, _, _), (lat2, lon2, _, _)= self.car_positions[i][-2:]
            x1, y1 = self.coords_to_local(lat1, lon1)
            x2, y2 = self.coords_to_local(lat2, lon2)
            points.append((x1, y1))
            points.append((x2, y2))

            if i == self.car - 1:
                initial_guess = [x2, y2, x2 - x1, y2 - y1]

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
        cte = np.arctan2(K*dist, v_ego)
        steer = heading_diff + cte
        return steer

    def mission_timer_callback(self, event):
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
            new_speed = v_ego + vel_accel * BROADCAST_INTERVAL
            new_speed = min(new_speed, SPEED_LIMIT)
            delta_rad = np.radians(delta)
            msg = Twist()
            msg.linear.x = -new_speed * math.sin(delta_rad)
            msg.linear.y = new_speed * math.cos(delta_rad)
            # self.publisher.publish(msg)
            self.movement_message = msg
        elif self.mission_status == DISARMING:
            msg = Twist()
            self.publisher.publish(msg)
        elif self.mission_status == MISSIONCOMPLETE:
            rospy.signal_shutdown("Mission completed successfully.")
        elif self.mission_status == ABORT:
            print("Aborting mission.")
            with open("datapoints.pkl", "wb") as f:
                pickle.dump(self.datapoints, f)
            self.datapoints = None
            emergency_disarm_req = CommandLong()
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
            self.killswitch_service(emergency_disarm_req)
            self.mission_status = MISSIONCOMPLETE

if __name__ == '__main__':
    car_number = int(input("Enter the car number: "))
    udp_publisher = UDPPublisher(car_number)
    rospy.spin()