import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
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

heading = None
latlon = None

def sat_callback(data):
	global latlon
	latlon = (data.latitude, data.longitude)
	# rospy.loginfo("Latitude: "+str(data.latitude)+", Longitude: "+str(data.longitude))

	# msg = json.dumps({"car": 0, "lat": data.latitude, "lon": data.longitude, "time": time(), "abort": False})

	# msg = msg.encode()
	# sock.sendto(msg, ('224.0.0.1', 5004))
 
def heading_listener_callback(data):
	global heading
	heading = data.data

def timer_callback(event):
	global latlon, heading
	if latlon is not None and heading is not None:
		rospy.loginfo("Latitude: "+str(latlon[0])+", Longitude: "+str(latlon[1])+", Heading: "+str(heading))

		msg = json.dumps({"car": 0, "lat": latlon[0], "lon": latlon[1], "head": heading, "time": time(), "abort": False})

		msg = msg.encode()
		sock.sendto(msg, ('224.0.0.1', 5004))

def sat_listener():
	rospy.init_node('satellite_listener', anonymous=True)
	# rospy.Subscriber('/fix', NavSatFix, sat_callback)
	rospy.Subscriber('/mavros/global_position/global', NavSatFix, sat_callback)
	rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, heading_listener_callback)
	rospy.Timer(rospy.Duration(0.1), timer_callback)
	rospy.spin()

if __name__ == '__main__':
	sat_listener()


