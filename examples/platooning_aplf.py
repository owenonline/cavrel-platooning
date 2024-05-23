import os
import sys
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, os.pardir)))
from OpenConvoy.crossplatform import ROSArgs
import argparse
import json

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
parser.add_argument('--follow_distance', type=float, default=2.0)
parser.add_argument('--speed_limit', type=float, default=5.0)

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
        follow_distance=args.follow_distance,
        speed_limit=args.speed_limit,
        car_number=args.car_number,
        car_length=args.car_length
    )

    # By default, platooning with APLF is implemented, so no subclass needs to be made

    if args.ros_version == 'ros1':
        from OpenConvoy.ros1_control import ROS1Control
        control = ROS1Control(rosargs)
    else:
        from OpenConvoy.ros2_control import ROS2Control
        import rclpy
        from rclpy.executors import MultiThreadedExecutor

        rclpy.init(args=None)
        control = ROS2Control(rosargs)
        executor = MultiThreadedExecutor()
        executor.add_node(control)
        executor.spin()