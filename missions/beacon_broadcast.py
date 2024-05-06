import socket
import struct
import pyproj
import json
import math
from time import time, sleep

EARTH_RADIUS = 6371e3
DUE_EAST = 90

# set up for broadcast
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

# set up for geodesic calculations
geodesic = pyproj.Geod(ellps='WGS84')

straight_1 = [(28.602225, -81.196822), (28.602224999845845, -81.19661752110925)]
turn_1 = [(28.602224999845845, -81.19661752110925), (28.60217988539375, -81.19661752110925)]
straight_2 = [(28.60217988539375, -81.19661752110925), (28.602179885239593, -81.19682199991266)]
turn_2 = [(28.602179885239593, -81.19682199991266), (28.602224999691693, -81.196821999912662)]
center_latitude = 28.602202442735443 
center_longitude = -81.19671976053279
center_orientation = DUE_EAST
target_speed = 3
broadcast_int = 0.1

seg_dist = target_speed*broadcast_int

def coords_to_local(target_lat, target_lon):
    # Convert lat, lon to radians
    target_lat_rad, target_lon_rad = math.radians(target_lat), math.radians(target_lon)
    current_lat_rad, current_lon_rad = math.radians(center_latitude), math.radians(center_longitude)

    x = EARTH_RADIUS * (target_lon_rad - current_lon_rad) * math.cos((current_lat_rad + target_lat_rad) / 2)
    y = EARTH_RADIUS * (target_lat_rad - current_lat_rad)

    angle = math.radians(center_orientation - DUE_EAST)
    qx = math.cos(angle) * x - math.sin(angle) * y
    qy = math.sin(angle) * x + math.cos(angle) * y
    return qx, qy

bearing_1, _, straight_1_dist = geodesic.inv(straight_1[0][1], straight_1[0][0], straight_1[1][1], straight_1[1][0])
print(f"bearing for straight 1: {bearing_1}")
straight_1_seg_count = int(straight_1_dist/seg_dist)
straight_1_segs = [straight_1[0]]
for frame in range(straight_1_seg_count):
    lat_prev, lon_prev = straight_1_segs[-1]
    lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing_1, seg_dist)
    straight_1_segs.append((lat_next, lon_next))

bearing_2, _, straight_2_dist = geodesic.inv(straight_2[0][1], straight_2[0][0], straight_2[1][1], straight_2[1][0])
print(f"bearing for straight 2: {bearing_2}")
straight_2_seg_count = int(straight_2_dist/seg_dist)
straight_2_segs = [straight_2[0]]
for frame in range(straight_2_seg_count):
    lat_prev, lon_prev = straight_2_segs[-1]
    lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing_2, seg_dist)
    straight_2_segs.append((lat_next, lon_next))

_, _, turn_1_dist = geodesic.inv(turn_1[0][1], turn_1[0][0], turn_1[1][1], turn_1[1][0])
turn_1_arclen = math.pi*(turn_1_dist/2)
turn_1_seg_count = int(turn_1_arclen/seg_dist)
bearing_diff_roc = (bearing_2 - bearing_1)/turn_1_seg_count
print(f"starting bearing for turn 1: {bearing_1}, ending bearing for turn 1: {bearing_2}, bearing diff: {bearing_diff_roc}")
turn_1_segs = [(turn_1[0][0], turn_1[0][1], bearing_1)]
for frame in range(turn_1_seg_count):
    lat_prev, lon_prev, bearing_prev = turn_1_segs[-1]
    bearing_next = bearing_prev - bearing_diff_roc
    lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing_next, seg_dist)
    turn_1_segs.append((lat_next, lon_next, bearing_next))

_, _, turn_2_dist = geodesic.inv(turn_2[0][1], turn_2[0][0], turn_2[1][1], turn_2[1][0])
turn_2_arclen = math.pi*(turn_2_dist/2)
turn_2_seg_count = int(turn_2_arclen/seg_dist)
bearing_diff_roc = (bearing_2 - bearing_1)/turn_2_seg_count
print(f"starting bearing for turn 2: {bearing_2}, ending bearing for turn 2: {bearing_1}, bearing diff: {bearing_diff_roc}")
turn_2_segs = [(turn_2[0][0], turn_2[0][1], bearing_2)]
for frame in range(turn_2_seg_count):
    lat_prev, lon_prev, bearing_prev = turn_2_segs[-1]
    bearing_next = bearing_prev - bearing_diff_roc
    lon_next, lat_next, _ = geodesic.fwd(lon_prev, lat_prev, bearing_next, seg_dist)
    turn_2_segs.append((lat_next, lon_next, bearing_next))

full_track = straight_1_segs + turn_1_segs + straight_2_segs + turn_2_segs

while True:
    for (lat, lon, head) in full_track:
        msg = json.dumps({"car": 0, "lat": lat, "lon": lon, "head": head, "time": time(), "abort": False})
        msg = msg.encode()
        sock.sendto(msg, ('224.0.0.1', 5004))
        sleep(broadcast_int)