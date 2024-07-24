dfrom scipy.optimize import minimize, least_squares
from collections import defaultdict, deque
from datetime import datetime
from time import time
import numpy as np
import socket
import struct
import json
import random
import math
import pickle
import os

# set up mission states
MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4

# set up constants
EARTH_RADIUS = 6371e3
DUE_NORTH = 0


class ROSArgs:
    def __init__(self, car_number, wheelbase, follow_distance=None, speed_max=None, speed_min=None, accel_max=None, accel_min=None, steer_max=None, steer_min=None,
                 broadcast_interval=None, listen_interval=None, drop_rate=None, track_name=None, center_lat=None, center_lon=None, center_orientation=None, heading_con_type=None, speed_con_type=None, 
                 P_Kp=None, I_Ki=None, PI_Kp=None, PI_Ki=None, PD_Kp=None, PD_Kd=None, PID_Kp=None, PID_Ki=None, PID_Kd=None, k=None, ks=None, save_path=None, **kwargs):

        self.car_number = car_number
        self.wheelbase = wheelbase

        self.follow_distance = follow_distance
        self.speed_max = speed_max    # +5 m/s
        self.speed_min = speed_min    # 0 m/s
        self.accel_max = accel_max    # +1.0
        self.accel_min = accel_min    # -1.0
        self.steer_max = steer_max    #unit: degree
        self.steer_min = steer_min    #unit: degree
        
        self.broadcast_interval = broadcast_interval
        self.listen_interval = listen_interval
        self.drop_rate = drop_rate

        self.track_name = track_name
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.center_orientation = center_orientation

        self.heading_con_type = heading_con_type
        self.speed_con_type = speed_con_type
        self.P_Kp = P_Kp
        self.I_Ki = I_Ki
        self.PI_Kp = PI_Kp
        self.PI_Ki = PI_Ki
        self.PD_Kp = PD_Kp
        self.PD_Kd = PD_Kd
        self.PID_Kp = PID_Kp
        self.PID_Ki = PID_Ki
        self.PID_Kd = PID_Kd

        self.k = k    # default is 0.3
        self.ks = ks  # default is 20.0

        self.save_path = save_path
       
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
        self.error_buffer = deque(maxlen=10)
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
            speed=np.abs(self.telem.twist.twist.linear.x),
            heading=self.heading.data,
            acceleration=np.sqrt(self.imu.linear_acceleration.x**2 + self.imu.linear_acceleration.y**2),
            yaw_rate=self.imu.angular_velocity.z,
            event_flags={
                'abort': self.mission_status in [DISARMING, MISSIONCOMPLETE],
                'car': self.args.car_number,
            }
        )


    def should_broadcast(self):
        """Determines whether the car should broadcast a message. If you want to implement any kind of event triggered communication, override this function in your subclass to do so."""
        return True


    def should_listen(self, sender_number):
        """Determines whether the car should listen to the broadcast from the sender based on the car's position in the platoon.
        Override this function in your subclass to implement a different network topology, or leave as-is for APLF."""
        return sender_number < self.args.car_number


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


    def _broadcast(self):
        """ Broadcasts the car's current GPS position and heading to all other cars in the network, dropping packets at a specified rate. """
        # Preconditions check
        if self.satellite is None or self.heading is None or (self.args.drop_rate > 0 and random.random() < self.args.drop_rate):
            return
        
        # Broadcasting
        if self.should_broadcast():
            msg = self.state.to_json()
            msg = msg.encode()
            self.broadcast_sock.sendto(msg, ('224.0.0.1', 5004))


    def _listen(self):
        """ Listens for and stores the latest broadcasts from other cars in the network. """
        
        # Receiving and decoding data 
        data, _ = self.listen_sock.recvfrom(1024)
        data_json = json.loads(data.decode())

        # Creating a message object
        bsm_message = BasicSafetyMessage(**data_json)

        # Processing the message
        if bsm_message.event_flags['abort'] and not self.mission_status in [DISARMING, MISSIONCOMPLETE]:
            self._disarm()
        
        if self.should_listen(bsm_message.event_flags['car']):
            self.car_positions[bsm_message.event_flags['car']].append(bsm_message)
            self.car_positions[bsm_message.event_flags['car']] = self.car_positions[bsm_message.event_flags['car']][-4:]

        # Determine that the beacon is broadcasting if we have a history of 2 messages from it
        if len(self.car_positions[0]) >= 2:
            self.beacon_started = True



    def coords_to_local(self, target_lat, target_lon):
        """  Converts GPS coordinates to local cartesian coordinates with respect to the track center point. """

        # Convert degrees to radians
        phi1 = math.radians(self.args.center_lat)
        lambda1 = math.radians(self.args.center_lon)
        phi2 = math.radians(target_lat)
        lambda2 = math.radians(target_lon)

        # Equirectangular approximation to convert to local Cartesian coordinates
        x = EARTH_RADIUS * (lambda2 - lambda1) * math.cos((phi1 + phi2) / 2)
        y = EARTH_RADIUS * (phi2 - phi1)

        # Adjust for the center orientation angle  (Center orientation is the angle of y axis of local cartesian) 
        theta_rad = math.radians(DUE_NORTH - self.args.center_orientation)    
        qx = math.cos(theta_rad) * x - math.sin(theta_rad) * y
        qy = math.sin(theta_rad) * x + math.cos(theta_rad) * y

        return qx, qy
    

   
    def speed_controller(self, v_ego, v_desired, dt):
        """   Calculates accelaration using various speed controllers.  """

        # Calculate velocity error
        error = v_desired - v_ego

        # Append current error to the error buffer
        self.error_buffer.append(error)

        # Calculate derivative and integral of error if sufficient buffer length
        if len(self.error_buffer) >= 2:
            D_error = (self.error_buffer[-1] - self.error_buffer[-2]) / dt
            I_error = sum(self.error_buffer) * dt
        else:
            D_error = 0
            I_error = 0

        # Select acceleration control law based on controller type  
        if self.args.speed_con_type == 'P':
            accel = self.args.P_Kp * error
        elif self.args.speed_con_type == 'I':
            accel = self.args.I_Ki * I_error
        elif self.args.speed_con_type == 'PI':
            accel = (self.args.PI_Kp * error) + (self.args.PI_Ki * I_error)
        elif self.args.speed_con_type == 'PD':
            accel = (self.args.PD_Kp * error) + (self.args.PD_Kd * D_error)
        elif self.args.speed_con_type == 'PID':
            accel = (self.args.PID_Kp * error) + (self.args.PID_Ki * I_error) + (self.args.PID_Kd * D_error)
        else:
            raise ValueError("Invalid controller type")

        # Clip acceleration to defined bounds
        # accel = np.clip(accel, self.args.accel_min, self.args.accel_max)

        return accel


    
    def distance_to_St_line(self, x0, y0, dx, dy, x, y):
        """           Calculates the distance between a point (x,y) and            """
        """    a line defined by a point (x0,y0) and a direction vector (dx,dy).   """
        
        # Vector from (x0, y0) to (x, y)
        vector_to_point = np.array([x - x0, y - y0])
      
        # Direction vector of the line
        line_direction = np.array([dx, dy])
    
        # Calculate the proportion of the direction vector where the closest point lies
        lambda_val = np.dot(vector_to_point, line_direction) / np.dot(line_direction, line_direction)
    
        # Calculates the coordinates of the closest point on the line to (x, y)
        closest_point = np.array([x0 + lambda_val * dx, y0 + lambda_val * dy])

        # Computes the Euclidean distance between closest point and point (x,y)
        distance = np.linalg.norm(closest_point - np.array([x, y]))

        # Determine the sign of the distance based on the relative position of (x, y) to the line
        cross_product = np.cross(vector_to_point, line_direction)
        
        if cross_product > 0:
            distance = distance  # Point (x,y) is to the right of line
        elif cross_product < 0:
            distance = -distance   # Point (x,y) is to the left of line
        else:
            distance = 0          # Point (x,y) is exactly on the line

        return distance



    def distance_to_curve_line(self, poly_coeffs, x, y):
        """  Calculates the distance between a point (x,y) and a polynominal curve.  """
        
        # Polynomial function
        poly_func = np.poly1d(poly_coeffs)

        # Function to calculate the distance between a point (x0, y0) and the curve at (x_curve, y_curve)
        def distance_function(x_curve):
            y_curve = poly_func(x_curve)
            return np.sqrt((x - x_curve)**2 + (y - y_curve)**2)

        # Find the x_curve that minimizes the distance function
        closest_point = minimize(distance_function, x)
        x_nearest = closest_point.x[0]
        y_nearest = poly_func(x_nearest)

        # Calculate the Euclidean distance between (x, y) and the nearest point on the curve
        distance = np.sqrt((x - x_nearest)**2 + (y - y_nearest)**2)
      
        # Determine the sign of the distance based on the relative position of (x, y) to the curve
        radius_curve = np.sqrt(x_nearest**2 + y_nearest**2)
        radius_point = np.sqrt(x**2 + y**2)
        if radius_curve < radius_point:
            distance = distance  # Point (x,y) is to the right of the curve
        elif radius_curve > radius_point:
            distance = -distance  # Point (x,y) is to the left of the curve
        else:
            distance = 0  # Point (x,y) is exactly on the curve
    
        return distance


    
    def heading_controller(self, v_ego, yaw_ego, yaw_desired):
        """     Calculates the steering angle using various heading controllers    """
       
        # Select heading control law based on controller type 
        if self.args.heading_con_type in ['Stanley_Straight', 'Stanley_Curve']:
        
            # 1. Calculate heading error
            yaw_diff = yaw_desired - yaw_ego

            if yaw_diff > 180.0:
                yaw_diff -= 360.0
            elif yaw_diff < -180.0:
                yaw_diff += 360.0

            # 2. Calculate crosstrack error
            points = []
            initial_guess = None

            # Get the local coordinates of ego vehicle
            x1_ego, y1_ego = self.coords_to_local(self.state.latitude, self.state.longitude)

            if self.args.heading_con_type == 'Stanley_Straight':
                
                # Get the local coordinates of the last two locations of all other cars
                for key, value in self.car_positions.items():
                  #  if len(value) < 2:
                  #     print(f"Error: Car {key} does not have enough position data.")
                  #     return None

                    m1, m2 = value[-2:]
                    x1, y1 = self.coords_to_local(m1.latitude, m1.longitude)
                    x2, y2 = self.coords_to_local(m2.latitude, m2.longitude)
                    points.append((x1, y1))
                    points.append((x2, y2))

                    # Use the car closest to the ego vehicle as the initial guess for the line (straight fitting) to ensure convergence
                    if key == self.args.car_number - 1:
                        initial_guess = [x1, y1, x2 - x1, y2 - y1]
        
                if initial_guess is None:
                    print("Error: No valid initial guess for straight line fitting.")
                    
                # Function to create a list of distances to a straight line
                def distances_to_st_line(params, points):
                    x0, y0, dx, dy = params
                    distances = []
                    for (x, y) in points:
                        distance = self.distance_to_St_line(x0, y0, dx, dy, x, y)
                        distances.append(distance)
                    return distances

                # compute a line of best fit using the last 2 positions of all other cars
                result = least_squares(distances_to_st_line, initial_guess, args=(points,))
                x0_opt, y0_opt, dx_opt, dy_opt = result.x

                # Calculate crosstrack error value
                cte = self.distance_to_St_line(x0_opt, y0_opt, dx_opt, dy_opt, x1_ego, y1_ego)

            elif self.args.heading_con_type == 'Stanley_Curve':

                # Get the local coordinates of the last two locations of all other cars
                for key, value in self.car_positions.items():
                 #   if len(value) < 2:
                 #      print(f"Error: Car {key} does not have enough position data.")
                 #      return None

                    m1, m2 = value[-2:]
                    x1, y1 = self.coords_to_local(m1.latitude, m1.longitude)
                    x2, y2 = self.coords_to_local(m2.latitude, m2.longitude)
                    points.append((x1, y1))
                    points.append((x2, y2))

             ##       # Use the car closest to the ego vehicle as the initial guess for the curve fitting
             ##       if key == self.args.car_number - 1:
             ##           initial_guess = [0, (y2-y1)/(x2-x1), y1-(x1*(y2-y1))/(x2-x1)]
        
             ##   if initial_guess is None:
             ##       print("Error: No valid initial guess for curve fitting.")
             ##       return None

             ##   # Function to create a list of distances to the curve
             ##   def distances_to_curve(params, points):
             ##       distances = []
             ##       for (x, y) in points:
             ##           distance = self.distance_to_curve_line(params, x, y)
             ##           distances.append(distance)
             ##       return distances

             ##   # Compute a quadratic curve of best fit using the last 2 positions of all other cars
             ##   result = least_squares(distances_to_curve, initial_guess, args=(points,))
             ##   a_opt, b_opt, c_opt = result.x

             ##   # Calculate crosstrack error value
             ##   cte = self.distance_to_curve_line([a_opt, b_opt, c_opt], x1_ego, y1_ego)
                
                if not points:
                    print("Error: No valid points for curve fitting.")
                    return None

                # Extract x and y coordinates from points
                x_coords, y_coords = zip(*points)

                # Fit a polynomial curve (e.g., quadratic curve) to the points
                poly_coeffs = np.polyfit(x_coords, y_coords, 2)  # Change the degree if needed

                # Calculate crosstrack error value
                cte = self.distance_to_curve_line(poly_coeffs, x1_ego, y1_ego)  


            # Calculate the removal angle of crosstrack error
            yaw_diff_crosstrack = np.arctan((self.args.k * cte) / (self.args.ks + v_ego))
            yaw_diff_crosstrack = np.rad2deg(yaw_diff_crosstrack)
            print("heading error: {yaw_diff} crosstrack error: {yaw_diff_crosstrack} heading: {egohead} ego pos: ({x1_ego}, {y1_ego})".format(
                      yaw_diff=yaw_diff, yaw_diff_crosstrack=yaw_diff_crosstrack, egohead=yaw_ego, x1_ego=x1_ego, y1_ego=y1_ego))

            # 3. Control law
            steer_expect = yaw_diff + yaw_diff_crosstrack
    
            if steer_expect > 180.0:
                steer_expect -= 360.0
            elif steer_expect < -180.0:
                steer_expect += 360.0

            steer_expect = np.clip(steer_expect, self.args.steer_min, self.args.steer_max)

            # steering angle scaling in (-1.0, 1.0)
            # steer_expect = steer_expect/steer_scale
            # steer_expect = max(min(steer_expect, 1.0), -1.0)

        # Presents steering angle in degree
        return steer_expect


        
    def minimization_objective(self, params):
        """                     Cost function to minimize to implement cooperative driving policy.                     """ 
        """ By default implements platooning. Override this function in your subclass to implement a different policy. """
        # Initialization
        v_ego = params[0]
        yaw_ego = params[1]
        yaw_ego = np.radians(yaw_ego)

        # Ego vehicle coordinate transform
        x, y = self.coords_to_local(self.state.latitude, self.state.longitude)

        # Cost Calculation
        total_cost = 0
        for _, value in self.car_positions.items():
            value = value[-1]
            x_target, y_target = self.coords_to_local(value.latitude, value.longitude)
            position = self.args.car_number - value.event_flags['car']
            v_target, yaw_target = value.speed, np.radians(value.heading)
            
            # Calculation goal follow distance
            goal_follow_distance = self.args.follow_distance * position

            # Simulation future positions
            x_target_future = x_target - (v_target * np.sin(yaw_target) * self.args.broadcast_interval)
            y_target_future = y_target + (v_target * np.cos(yaw_target) * self.args.broadcast_interval)
            x_ego_future = x - (v_ego * np.sin(yaw_ego) * self.args.broadcast_interval)
            y_ego_future = y + (v_ego * np.cos(yaw_ego) * self.args.broadcast_interval)

            # Calculation of goal position
            x_ego_goal = x_target_future + (goal_follow_distance * np.sin(yaw_target))
            y_ego_goal = y_target_future - (goal_follow_distance * np.cos(yaw_target))

            # print("goal follow: {gfd}, head target: {th}, vtarget: {vt}, selected {v} {h}".format(gfd=goal_follow_distance, th=value.heading, vt=value.speed, v=v, h=np.rad2deg(yaw_ego)))
            
            # Cost computation 
            total_cost += np.sqrt((x_ego_goal - x_ego_future)**2 + (y_ego_goal - y_ego_future)**2)

        return total_cost



    def _get_goal_motion(self):
        """  Calculates the target speed and heading for the ego vehicle based on the positions of the other cars in the network.  """ 
        """                 (Uses the implemented minimization objective, which by default is for platooning)                      """
       
        # bounds = [(0, 4)]
        # head, v = self.car_positions[0][-1].heading, self.car_positions[0][-1].speed
        # response = minimize(lambda params: self.minimization_objective(params), [v], method='SLSQP', bounds=bounds)

        bounds = [(0, 4), (-180, 180)]
        v, yaw = self.car_positions[0][-1].speed, self.car_positions[0][-1].heading
        response = minimize(lambda params: self.minimization_objective(params), [v, yaw], method='SLSQP', bounds=bounds)
        print("calculated target v {v} and yaw {yaw}".format(v=v, yaw=yaw))
        
        return response

    def _get_applied_motion(self, v_desired, yaw_desired):
        """  Calculates the forward/backward velocity and yaw rate for vehicle base on its desired speed and heading.  """
        # Getting current speed and heading and time interval
        v_ego = self.state.speed
        yaw_ego = self.state.heading
        dt = self.args.broadcast_interval

        # Speed Calculation
        throttle = self.speed_controller(v_ego, v_desired, dt)
        new_velocity = v_ego + (throttle * dt)
        new_velocity = np.clip(new_velocity, self.args.speed_min, self.args.speed_max)
        #new_velocity = min(new_velocity, self.args.speed_max)

        # Heading Calculation
        new_steering = self.heading_controller(v_ego, yaw_ego, yaw_desired)
        new_steering = np.radians(new_steering)

        # Calculates the forward/backward velocity and yaw rate
        velocity_x = new_velocity
        angular_z = (new_velocity / self.args.wheelbase) * np.tan(new_steering)
        print("applying throttle {throttle}, speed {speed} and steering {steering} with yaw rate {yaw_rate}".format(throttle=throttle, speed=velocity_x, steering=new_steering, yaw_rate=angular_z))

        return velocity_x, angular_z



    def _update_datapoints(self):
        """                   Updates the all car datapoints                       """
        # Initialization
        to_add = []

        # Collecting data from other cars
        for _, value in self.car_positions.items():
            value = value[-1]
            x, y = self.coords_to_local(value.latitude, value.longitude)
            to_add.append((x, y, value.heading, value.speed, value.acceleration, time()))

        # Collecting data from ego car
        ex, ey = self.coords_to_local(self.state.latitude, self.state.longitude)
        to_add.append((ex, ey, self.state.heading, self.state.speed, self.state.acceleration, time()))

        # Updating datapoints
        self.datapoints.append(to_add)



    def _save_data(self):
        """           Saves the current datapoints of the car to a file.           """

        # Getting the current date and time
        now = datetime.now()
        formatted_date = now.strftime('%m_%d')
        formatted_time = now.strftime('%H_%M')

        # Creating the save path
        save_path = os.path.join(self.args.save_path, self.args.track_name, formatted_date)
        # Creating the directory if it doesnot exist
        if not os.path.exists(save_path):
            os.makedirs(save_path)
        # Creating the file name
        file_name = "py2_{car}_{time}_{droprate}.pkl".format(car=self.args.car_number, time=formatted_time, droprate=self.args.drop_rate)
        # Creating the full file path
        file_path = os.path.join(save_path, file_name)

        # Saving the data to a file        
        with open(file_path, "wb") as f:
            pickle.dump(self.datapoints, f)


