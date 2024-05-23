import rospy
import struct
import socket
import json
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandLong
from time import sleep, time
import threading
import math

# set up mission states
MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4
ABORT = -1

SPEED_LIMIT = 2.2
MAX_STEER = 30


class UDPPublisher:
    def __init__(self):
        rospy.init_node('udp_publisher')

        self.mission_status = MISSIONSTART
        self.pressed_keys = set()

        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind(('', 5004))
        group = socket.inet_aton('224.0.0.1')
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self.listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        rospy.Timer(rospy.Duration(0.01), self.listen_timer_callback)

        # publisher setup
        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=20)

        rospy.Timer(rospy.Duration(0.1), self.mission_timer_callback)

        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/command')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.killswitch_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        self.rate = rospy.Rate(20)
        self.move_thread = threading.Thread(target=self.movement_callback)
        self.move_thread.daemon = True
        self.move_thread.start()

    def listen_timer_callback(self, event):
        data, _ = self.listen_sock.recvfrom(1024)
        data_json = json.loads(data.decode())
        self.pressed_keys = set(data_json['keys'])

    def movement_callback(self):
        """Publishes movement commands at the rate required to keep the vehicle armed"""

        while True:
            msg = Twist()

            new_speed = 0
            delta = 0

            if 'w' in self.pressed_keys:
                new_speed = SPEED_LIMIT
            elif 's' in self.pressed_keys:
                new_speed = -SPEED_LIMIT

            if 'a' in self.pressed_keys:
                delta = MAX_STEER
            elif 'd' in self.pressed_keys:
                delta = -MAX_STEER

            if 'p' in self.pressed_keys:
                self.mission_status = ABORT
                return
        
            delta = math.radians(delta)

            msg.linear.x = -new_speed * math.sin(delta)
            msg.linear.y = new_speed * math.cos(delta)

            self.publisher.publish(msg)
            self.rate.sleep()

    def mission_timer_callback(self, event):
        """Main loop for vehicle control. Handles the arming, moving, and disarming of the rover."""

        if self.mission_status == MISSIONSTART:
            print("Switching to offboard mode.")
            try:
                response = self.set_mode_service(0, 'OFFBOARD')
                if response.mode_sent:
                    print("arming")
                    self.mission_status = ARMING
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
        elif self.mission_status == ARMING:
            try:
                sleep(4)
                response = self.arming_service(True)
                if response.success:
                    print("Moving")
                    self.mission_status = MOVING
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
        elif self.mission_status == MISSIONCOMPLETE:
            rospy.signal_shutdown("Mission completed successfully.")
        elif self.mission_status == ABORT:
            print("Aborting mission.")

            self.mission_status = DISARMING
            response = self.arming_service(False)
            if response.success:
                print("Disarmed")
                self.mission_status = MISSIONCOMPLETE

if __name__ == '__main__':
    udp_publisher = UDPPublisher()
    rospy.spin()