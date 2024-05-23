import os
import sys
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, os.pardir)))
from OpenConvoy.crossplatform import ROSArgs
import argparse
import json
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('ros_version', type=str, choices=['ros1', 'ros2'])
parser.add_argument('--car_number', type=int, default=1, help='position of the car in the platoon. 0 is reserved for the beacon.')
parser.add_argument('--car_length', type=float, default=0.779, help='length of the cars in front of the current vehicle. They must be the same length.')
parser.add_argument('--broadcast_interval', type=float, default=0.1)
parser.add_argument('--listen_interval', type=float, default=0.01)
parser.add_argument('--drop_rate', type=float, default=0.0)
parser.add_argument('--track_path', type=str)
parser.add_argument('--save_path', type=str, default='data')
parser.add_argument('--kpv', type=float, default=0.3)
parser.add_argument('--kdv', type=float, default=1.1)
parser.add_argument('--k', type=float, default=0.2)
parser.add_argument('--ks', type=float, default=0.1)
parser.add_argument('--speed_limit', type=float, default=5.0)
parser.add_argument('--follow_time', type=float, default=2.0)

if __name__ == '__main__':
    args = parser.parse_args()

    with open(args.track_path, 'r') as f:
        track = json.load(f)
        args.track_name = track['name']
        args.center_lat = track['center']['lat']
        args.center_lon = track['center']['lon']
        args.center_orientation = 90.0

    rosargs = ROSArgs(
        ros_version=args.ros_version,
        broadcast_interval=args.broadcast_interval,
        listen_interval=args.listen_interval,
        drop_rate=args.drop_rate,
        track_name=args.track_name,
        center_lat=args.center_lat,
        center_lon=args.center_lon,
        center_orientation=args.center_orientation,
        save_path=args.save_path,
        kpv=args.kpv,
        kdv=args.kdv,
        k=args.k,
        ks=args.ks,
        speed_limit=args.speed_limit,
        car_number=args.car_number,
        car_length=args.car_length,
        follow_time=args.follow_time,
    )

    # By default, platooning with APLF is implemented, so no subclass needs to be made

    if args.ros_version == 'ros1':
        from OpenConvoy.ros1_control import ROS1Control as roscontrol
    else:
        from OpenConvoy.ros2_control import ROS2Control as roscontrol

    class cacc_aplf(roscontrol):
        def __init__(self, rosargs):
            super(cacc_aplf, self).__init__(rosargs)

        def minimization_objective(self, params):
            """Overriding the default platooning implementation to implement CACC"""

            v, head = params
            head = np.radians(head)
            x, y = self.coords_to_local(self.state.latitude, self.state.longitude)

            total_cost = abs(v - self.state.speed)
            for _, value in self.car_positions.items():
                value = value[-1]
                x_target, y_target = self.coords_to_local(value.latitude, value.longitude)
                goal_follow_time = self.args.follow_time * (self.args.car_number - value.event_flags['car']) # how far should I be behind any specific car
                head_target, v_target = np.radians(value.heading), value.speed

                x_sim_target = x_target + v_target*np.sin(head_target)*self.args.broadcast_interval
                y_sim_target = y_target + v_target*np.cos(head_target)*self.args.broadcast_interval
                x_sim_ego = x + v*np.sin(head)*self.args.broadcast_interval
                y_sim_ego = y + v*np.cos(head)*self.args.broadcast_interval

                sim_distance = np.sqrt((x_sim_ego - x_sim_target)**2 + (y_sim_ego - y_sim_target)**2)
                sim_time = sim_distance / v

                total_cost += abs(sim_time - goal_follow_time)

            return total_cost
        
    if args.ros_version == 'ros1':
        control = cacc_aplf(rosargs)
    else:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        rclpy.init(args=None)
        control = cacc_aplf(rosargs)
        executor = MultiThreadedExecutor()
        executor.add_node(control)
        executor.spin()