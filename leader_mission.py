import rospy
from sensor_msgs.msg import NavSatFix
# import pyproj
# from sklearn.metrics.pairwise import haversine_distances
# from math import radians, cos, sin, sqrt, atan2
from time import time
import socket
import json

EARTH_RADIUS = 6371e3 # earth radius in meters
# geodesic = pyproj.Geod(ellps='WGS84')

client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
client_sock.bind(("", 37020))

# saved_point = None

def sat_callback(data):
	# lat1, lon1, time1 = saved_point
	# lat2, lon2, time2 = radians(data.latitude), radians(data.longitude), time()

	# TODO: make sure that the heading and distance returned here is consistent with what the pixhawk is putting out
	# heading, _, distance = geodesic.inv(lon1, lat1, lon2, lat2)
	# velocity = distance/(time2-time1)

	rospy.loginfo("Latitude: "+str(data.latitude)+", Longitude: "+str(data.longitude))#+", Heading: "+str(heading)+", Distance: "+str(distance)+", Velocity: "+str(velocity))

	msg = json.dumps({"car": 0, "lat": data.latitude, "lon": data.longitude, "time": time(), "abort": False})#, "heading": heading, "distance": distance, "velocity": velocity, "time": time2})
	msg = msg.encode()

	client_sock.sendto(msg, ("255.255.255.255", 37020))

def sat_listener():
	rospy.init_node('satellite_listener', anonymous=True)
	rospy.Subscriber('/fix', NavSatFix, sat_callback)
	rospy.spin()

if __name__ == '__main__':
	sat_listener()


