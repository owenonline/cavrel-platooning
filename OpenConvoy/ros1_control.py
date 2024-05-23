from crossplatform import ROSArgs, Control
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode
import threading
from time import sleep

# set up mission states
MISSIONSTART = 0
ARMING = 1
MOVING = 2
DISARMING = 3
MISSIONCOMPLETE = 4
ABORT = -1

class ROS1Control(Control):
    def __init__(self, args):
        super(ROS1Control, self).__init__(args)
        rospy.init_node('udp_publisher')

        self.movement_message = Twist()

        # pubsub setup
        self.telem_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self._telem_listener_callback)
        self.satellite_subscriber = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self._satellite_listener_callback)
        self.heading_subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self._heading_listener_callback)
        self.accel_subscriber = rospy.Subscriber('/mavros/imu/data', Imu, self._accel_listener_callback)
        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=20)

        # arming and disarming service setup
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        # set up movement message broadcasting
        self.rate = rospy.Rate(20)
        self.move_thread = threading.Thread(target=self.__movement_callback)
        self.move_thread.daemon = True
        self.move_thread.start()

        # set up killswitch
        self.stop_thread = threading.Thread(target=self.__listen_for_stop)
        self.stop_thread.daemon = True
        self.stop_thread.start()

        # Timed components
        rospy.Timer(rospy.Duration(self.args.broadcast_interval), lambda event: self._broadcast())
        rospy.Timer(rospy.Duration(self.args.listen_interval), lambda event: self._listen())
        rospy.Timer(rospy.Duration(self.args.broadcast_interval), lambda event: self.__mission_timer_callback())

        rospy.spin()

    def __listen_for_stop(self):
        """Kills the mission if the user presses ENTER."""
        
        while True:
            raw_input()
            self._save_data()
            self._disarm()

    def __movement_callback(self):
        """Publishes movement commands at the rate required to keep the vehicle armed"""

        while True:
            self.publisher.publish(self.movement_message)
            self.rate.sleep()

    def __mission_timer_callback(self):
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
                    self.mission_status = MOVING
                else:
                    print(response)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        elif self.mission_status == MOVING:
            print("Moving")
            msg = Twist()

            if self.beacon_started and self.sensors_ok:
                self._update_datapoints()

                targets = self._get_goal_motion()

                if not targets.success:
                    print("Optimization failed:", targets.message)
                    return
                
                v, head = targets.x
                print("calculated target v {v} and heading {head}".format(v=v, head=head))

                msg.linear.x, msg.linear.y = self._get_applied_motion(v, head)

            self.movement_message = msg

        elif self.mission_status == MISSIONCOMPLETE:
            rospy.signal_shutdown("Mission completed successfully.")

    def _disarm(self):
        """Disarm the vehicle"""

        self.mission_status = DISARMING
        self.movement_message = Twist()
        disarmed = False
        while not disarmed:
            response = self.arming_service(False)
            if response.success:
                disarmed = True
                print("Disarmed")
        self.mission_status = MISSIONCOMPLETE