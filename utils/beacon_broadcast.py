import random
import socket
import struct
import pyproj
import json
import math
from time import time, sleep
import argparse
import threading
import numpy as np
import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, os.pardir)))
from OpenConvoy.crossplatform import BasicSafetyMessage

# constants
EARTH_RADIUS = 6371e3
DUE_EAST = 90
ABORT = False

# set up for broadcast
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

# set up for geodesic calculations
geodesic = pyproj.Geod(ellps='WGS84')

# set up args
parser = argparse.ArgumentParser()
parser.add_argument('--track_path', type=str, default="missions/tracks/bus_loop_large.json")
parser.add_argument('--broadcast_int', type=float, default=0.1)
parser.add_argument('--drop_rate', type=float, default=0.0)

def calculate_straight(start, end, seg_dist, speed):
    bearing, _, dist = geodesic.inv(start[1], start[0], end[1], end[0])
    seg_count = int(dist/seg_dist)
    segs = [(start[0], start[1], bearing, speed)]
    for _ in range(seg_count):
        lat_prev, lon_prev, _ , _= segs[-1]
        lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing, seg_dist)
        segs.append((lat_next, lon_next, bearing, speed))
    return segs, bearing

def calculate_turn(start, start_bearing, end, end_bearing, seg_dist, direction='left'):
    """Treats the start and end as vertices of the ellipse along its major and minor axes, and calculates the curge
    between them. The curve is calculated by incrementing the angle along the ellipse and calculating the distance.
    This method only works for turns up to 90 degrees."""

    bearing, _, dist = geodesic.inv(start[1], start[0], end[1], end[0])

    # determine starting angle along ellipse
    # compass angle being in degrees, and ellipse in radians
    if direction == 'left':
        c2e = lambda c_bearing: np.deg2rad((360 - c_bearing) % 360)
        e2c = lambda e_angle: (360 - np.rad2deg(e_angle)) % 360
        anglediff = lambda angle_1, angle_2: (angle_2 - angle_1 + np.pi*2) % (np.pi*2)
        increment = 0.001
    else:
        # TODO: implement support for right turns
        raise NotImplementedError("Right turns aren't supported yet")
    
    bearing = c2e(bearing)
    x_mult = abs(np.cos(bearing)*dist)
    y_mult = abs(np.sin(bearing)*dist)

    t_start = c2e(start_bearing)
    t_end = c2e(end_bearing)
    turn = anglediff(t_start, t_end)
    progress = 0

    # print(f"start_bearing: {start_bearing}, end_bearing: {end_bearing}")
    # print(f"t_start: {np.rad2deg(t_start)}, t_end: {np.rad2deg(t_end)}")

    points = [(x_mult*np.sin(t_start), y_mult*np.cos(t_start))]
    full_track = [(*start, start_bearing)]
    t = t_start

    while progress < turn:
        t = (t + increment) % (np.pi*2)
        progress += increment

        x = x_mult*np.sin(t)
        y = y_mult*np.cos(t)

        lx, ly = points[-1]
        dist = np.sqrt((x - lx)**2 + (y - ly)**2)
        if dist >= seg_dist:
            points.append((x, y))

            bearing = np.rad2deg(np.arctan2(y - ly, x - lx))
            lon_next, lat_next, _ = geodesic.fwd(full_track[-1][1], full_track[-1][0], bearing, dist)
            full_track.append((lat_next, lon_next, e2c(t)))

    final_point = (x_mult*np.sin(t_end), y_mult*np.cos(t_end))
    if points[-1] != final_point:
        dist = np.sqrt((final_point[0] - points[-1][0])**2 + (final_point[1] - points[-1][1])**2)
        bearing = np.rad2deg(np.arctan2(final_point[1] - points[-1][1], final_point[0] - points[-1][0]))
        lon_next, lat_next, _ = geodesic.fwd(full_track[-1][1], full_track[-1][0], bearing, dist)
        full_track.append((lat_next, lon_next, e2c(t_end)))

    return full_track

def calculate_track(straights, turns, broadcast_interval):
    straight_points = []
    bearings = []
    for straight in straights:
        seg_dist = broadcast_interval*straight['speed']
        start = (straight['start']['lat'], straight['start']['lon'])
        end = (straight['end']['lat'], straight['end']['lon'])
        segs, bearing = calculate_straight(start, end, seg_dist, straight['speed'])
        straight_points.append(segs)
        bearings.append(bearing)

    # bearings = [(bearings[i], bearings[(i+1)%len(bearings)]) for i in range(len(bearings))]
    # turn_points = []
    # for turn, (start_bearing, end_bearing) in zip(turns, bearings):
    #     seg_dist = broadcast_interval*turn['speed']
    #     start = (turn['start']['lat'], turn['start']['lon'])
    #     end = (turn['end']['lat'], turn['end']['lon'])
    #     segs = calculate_turn(start, start_bearing, end, end_bearing, seg_dist, turn['direction'])
    #     turn_points.append(segs)

    # full_track = []
    # for straight, turn in zip(straight_points, turn_points):
    #     full_track += straight + turn
    full_track = []
    for straight in straight_points:
        full_track += straight

    return full_track

def listen_for_stop():
    global ABORT
    while True:
        input() # wait for the user to press enter
        ABORT = True

if __name__ == '__main__':
    args = parser.parse_args()
    with open(args.track_path, 'r') as f:
        track = json.load(f)

    full_track = calculate_track(track['straights'], track['turns'], args.broadcast_int)

    stop_thread = threading.Thread(target=listen_for_stop)
    stop_thread.daemon = True
    stop_thread.start()

    for (lat, lon, head, speed) in full_track:
        # randomly drop packets
        if args.drop_rate > 0 and random.random() < args.drop_rate:
            sleep(args.broadcast_int)
            continue

        # send the packet
        bsm_msg = BasicSafetyMessage(
            time=time(),
            latitude=lat,
            longitude=lon,
            speed=speed,
            heading=head,
            acceleration=0,
            event_flags={
                "abort": False,
                "car": 0,
            }
        )
        msg = bsm_msg.to_json()
        print(msg)
        msg = msg.encode()
        sock.sendto(msg, ('224.0.0.1', 5004))

        # wait for next broadcast interval
        sleep(args.broadcast_int)