import rospy
from sensor_msgs.msg import NavSatFix
# import pyproj
# from sklearn.metrics.pairwise import haversine_distances
# from math import radians, cos, sin, sqrt, atan2
from time import time
import socket
import struct
import json

EARTH_RADIUS = 6371e3 # earth radius in meters

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
ttl = struct.pack('b', 1)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

def sat_callback(data):
	rospy.loginfo("Latitude: "+str(data.latitude)+", Longitude: "+str(data.longitude))

	msg = json.dumps({"car": 0, "lat": data.latitude, "lon": data.longitude, "time": time(), "abort": False})

	msg = msg.encode()
	sock.sendto(msg, ('224.0.0.1', 5004))

def sat_listener():
	rospy.init_node('satellite_listener', anonymous=True)
	# rospy.Subscriber('/fix', NavSatFix, sat_callback)
	rospy.Subscriber('/mavros/global_position/global', NavSatFix, sat_callback)
	rospy.spin()

if __name__ == '__main__':
	sat_listener()


