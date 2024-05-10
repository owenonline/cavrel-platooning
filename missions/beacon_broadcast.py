import random
import socket
import struct
import pyproj
import json
import math
from time import time, sleep
import argparse
import threading

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
parser.add_argument('--track_path', type=str, default="missions/tracks/bus_loop.json")
parser.add_argument('--broadcast_int', type=float, default=0.1)
parser.add_argument('--drop_rate', type=float, default=0.0)

def calculate_straight(start, end, seg_dist):
    bearing, _, dist = geodesic.inv(start[1], start[0], end[1], end[0])
    seg_count = int(dist/seg_dist)
    segs = [(start[0], start[1], bearing)]
    for _ in range(seg_count):
        lat_prev, lon_prev, _ = segs[-1]
        lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing, seg_dist)
        segs.append((lat_next, lon_next, bearing))
    return segs, bearing

def calculate_turn(start, start_bearing, end, end_bearing, seg_dist, direction='left'):
    _, _, dist = geodesic.inv(start[1], start[0], end[1], end[0])
    arclen = math.pi*(dist/2)
    seg_count = int(arclen/seg_dist)
    bearing_diff_roc = abs((end_bearing - start_bearing)/seg_count)

    if direction == 'right':
        bearing_diff_roc *= -1

    segs = [(start[0], start[1], start_bearing)]
    for _ in range(seg_count):
        lat_prev, lon_prev, bearing_prev = segs[-1]
        bearing_next = bearing_prev - bearing_diff_roc
        lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing_next, seg_dist)
        segs.append((lat_next, lon_next, bearing_next))
    return segs

def calculate_track(straights, turns, broadcast_interval):
    straight_points = []
    bearings = []
    for straight in straights:
        seg_dist = broadcast_interval*straight['speed']
        start = (straight['start']['lat'], straight['start']['lon'])
        end = (straight['end']['lat'], straight['end']['lon'])
        segs, bearing = calculate_straight(start, end, seg_dist)
        straight_points.append(segs)
        bearings.append(bearing)

    bearings = [(bearings[i], bearings[(i+1)%len(bearings)]) for i in range(len(bearings))]
    turn_points = []
    for turn, (start_bearing, end_bearing) in zip(turns, bearings):
        seg_dist = broadcast_interval*turn['speed']
        start = (turn['start']['lat'], turn['start']['lon'])
        end = (turn['end']['lat'], turn['end']['lon'])
        segs = calculate_turn(start, start_bearing, end, end_bearing, seg_dist, turn['direction'])
        turn_points.append(segs)

    full_track = []
    for straight, turn in zip(straight_points, turn_points):
        full_track += straight + turn

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

    while True:
        for (lat, lon, head) in full_track:
            # randomly drop packets
            if args.drop_rate > 0 and random.random() < args.drop_rate:
                continue

            # send the packet
            msg = json.dumps({"car": 0, "lat": lat, "lon": lon, "head": head, "time": time(), "abort": ABORT})
            print(msg)
            msg = msg.encode()
            sock.sendto(msg, ('224.0.0.1', 5004))

            # wait for next broadcast interval
            sleep(args.broadcast_int)