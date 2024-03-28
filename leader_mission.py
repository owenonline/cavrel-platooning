import rospy
from sensor_msgs.msg import NavSatFix
import socket
import json

client_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
client_sock.bind(("", 37020))

def sat_callback(data):
	rospy.loginfo("Latitude: "+str(data.latitude)+", Longitude: "+str(data.longitude))

	msg = json.dumps({"car": 0, "lat": data.latitude, "lon": data.longitude})
	msg = msg.encode()

	client_sock.sendto(msg, ("255.255.255.255", 37020))

def sat_listener():
	rospy.init_node('satellite_listener', anonymous=True)
	rospy.Subscriber('/fix', NavSatFix, sat_callback)
	rospy.spin()

if __name__ == '__main__':
	sat_listener()


