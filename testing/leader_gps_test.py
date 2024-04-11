import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
# import pyproj
# from sklearn.metrics.pairwise import haversine_distances
# from math import radians, cos, sin, sqrt, atan2
from time import time
import socket
import struct
import json
import numpy as np
EARTH_RADIUS = 6371e3 # earth radius in meters

smoothed_points = []
points = []

def sat_callback(data):
	global points

	points.append([data.latitude, data.longitude])

	# if len(points) < 3:
	# 	points.append([data.latitude, data.longitude])
	# else:
	# 	new_point = np.average(np.array(points[-2:] + [[data.latitude, data.longitude]]), axis=0)
	# 	points.append(new_point)
	points_np = np.array(points)
	# mean_lat = np.mean(points_np[:,0])
	# mean_lon = np.mean(points_np[:,1])
	std_lat = np.std(points_np[-10:,0])
	std_lon = np.std(points_np[-10:,1])
	print("latitude std: "+str(std_lat)+", Longitude std: "+str(std_lon))

def sat_listener():
	rospy.init_node('satellite_listener', anonymous=True)
	rospy.Subscriber('/fix', NavSatFix, sat_callback)
	rospy.spin()

if __name__ == '__main__':
	sat_listener()


