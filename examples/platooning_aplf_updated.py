import sys
import os
import argparse
import json
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, os.pardir)))
from OpenConvoy.crossplatform import ROSArgs


parser = argparse.ArgumentParser()
parser.add_argument('ros_version', type=str, choices=['ros1', 'ros2'])
parser.add_argument('--car_number', type=int, default=1, help='position of the car in the platoon. 0 is reserved for the beacon.')
parser.add_argument('--wheelbase', type=float, default=0.33, help='distance between the front and rear axles')
parser.add_argument('--follow_distance', type=float, default=2.5)
parser.add_argument('--speed_max', type=float, default=5.0)
parser.add_argument('--speed_min', type=float, default=0.0)
parser.add_argument('--accel_max', type=float, default=1.0)
parser.add_argument('--accel_min', type=float, default=-1.0)
parser.add_argument('--steer_max', type=float, default=90.0)
parser.add_argument('--steer_min', type=float, default=-90.0)
parser.add_argument('--broadcast_interval', type=float, default=0.1)
parser.add_argument('--listen_interval', type=float, default=0.01)
parser.add_argument('--drop_rate', type=float, default=0.0)
parser.add_argument('--track_path', type=str)
parser.add_argument('--heading_con_type', type=str, choices=['PurePursuit_Straight', 'PurePursuit_Curve', 'Stanley_Straight', 'Stanley_Curve'])
parser.add_argument('--speed_con_type', type=str, choices=['P', 'I', 'PI', 'PD', 'PID'])
parser.add_argument('--P_Kp', type=float, default=1.0)
parser.add_argument('--I_Ki', type=float, default=0.3)
parser.add_argument('--PI_Kp', type=float, default=1.0)
parser.add_argument('--PI_Ki', type=float, default=0.3)
parser.add_argument('--PD_Kp', type=float, default=1.0)
parser.add_argument('--PD_Kd', type=float, default=0.001)
parser.add_argument('--PID_Kp', type=float, default=1.0)
parser.add_argument('--PID_Ki', type=float, default=0.3)
parser.add_argument('--PID_Kd', type=float, default=0.001)
parser.add_argument('--k', type=float, default=0.3)
parser.add_argument('--ks', type=float, default=1.0)
parser.add_argument('--save_path', type=str, default='data')


if __name__ == '__main__':
    args = parser.parse_args()

    with open(args.track_path, 'r') as f:
        track = json.load(f)
        args.track_name = track['name']
        args.center_lat = track['center']['lat']
        args.center_lon = track['center']['lon']
        args.center_orientation = 0.0

    rosargs = ROSArgs(
        ros_version=args.ros_version,   
        car_number=args.car_number,
        wheelbase=args.wheelbase,
        follow_distance=args.follow_distance,
        speed_max=args.speed_max,
        speed_min=args.speed_min,
        accel_max=args.accel_max,
        accel_min=args.accel_min,
        steer_max=args.steer_max,
        steer_min=args.steer_min,
        broadcast_interval=args.broadcast_interval,
        listen_interval=args.listen_interval,
        drop_rate=args.drop_rate,
        track_name=args.track_name,
        center_lat=args.center_lat,
        center_lon=args.center_lon,
        center_orientation=args.center_orientation,
        heading_con_type=args.heading_con_type, 
        speed_con_type=args.speed_con_type, 
        P_Kp=args.P_Kp,
        I_Ki=args.I_Ki,
        PI_Kp=args.PI_Kp,
        PI_Ki=args.PI_Ki,
        PD_Kp=args.PD_Kp,
        PD_Kd=args.PD_Kd,
        PID_Kp=args.PID_Kp,
        PID_Ki=args.PID_Ki,
        PID_Kd=args.PID_Kd,
        k=args.k,
        ks=args.ks,
        save_path=args.save_path
    )

    # Implementing control based on ROS version
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

        try:
            executor.spin()
        finally:
            executor.shutdown()
            control.destroy_node()
            rclpy.shutdown()
