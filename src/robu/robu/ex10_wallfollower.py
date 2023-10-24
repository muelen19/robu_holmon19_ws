#Datein importieren fürs senden und empfangen von Ros Datein.................................................
import math
import rclpy                                                        #Ros schnittstelle für Python

from std_msgs.msg import String                                                
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data

import numpy as np
from enum import IntEnum
#.............................................................................................................


#Globale Variablen............................................................................................
ROBOT_DIRECTION_FRONT_INDEX = 0
ROBOT_DIRECTION_RIGHT_FRONT_INDEX = 45
ROBOT_DIRECTION_RIGHT_INDEX = 90
ROBOT_DIRECTION_RIGHT_REAR_INDEX = 135
ROBOT_DIRECTION_REAR_INDEX = 180
ROBOT_DIRECTION_LEFT_REAR_INDEX = 225
ROBOT_DIRECTION_LEFT_INDEX = 270
ROBOT_DIRECTION_LEFT_FRONT_INDEX = 315
#.............................................................................................................


#Zustände definieren..........................................................................................
class WallFollowerStates(IntEnum):
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL = 2,
    WF_STATE_FOLLOWWALL = 3,
#.............................................................................................................


#Node.........................................................................................................
class WallFollower(rclpy.Node):

    #+++++ Konstruktor +++++
    def __init__ (self):                                            
        super().__init__('Wallfollower')
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile_sensor_data)

        #+++++ Variablen +++++
        self.left_dist = 999999.9
        self.leftfront_dist = 999999.9
        self.front_dist = 999999.9
        self.rightfront_dist = 999999.9
        self.right_dist = 999999.9
        self.rear_dist = 999999.9

        self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL

        self.forward_speed_wf_slow = 0.05 #m/s
        self.forward_speed_wf_fast = 0.1  #m/s

        self.turning_speed_wf_slow = 0.1  #rad/s
        self.turning_speed_wf_slow = 1.0  #rad/s

        self.dist_thresh_wf = 0.3         #m
        self.dist_hysteresis_wf = 0.02    #m

        self.timer = self.create_timer(0.2, self.timer_callback)


    def timer_callback(self):
        pass

    def scan_callback(self, msg):

        self.left_dist = msg.ranges[ROBOT_DIRECTION_LEFT_INDEX]
        self.leftfront_dist = msg.ramges[ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        self.front_dist = msg.ramges[ROBOT_DIRECTION_FRONT_INDEX]
        self.rightfront_dist = msg.ramges[ROBOT_DIRECTION_RIGHT_FRONT_INDEX]
        self.right_dist = msg.ramges[ROBOT_DIRECTION_RIGHT_INDEX]
        self.rear_dist = msg.ramges[ROBOT_DIRECTION_REAR_INDEX]

        print("ld: %.2f m" %self.left_dist,
              "lfd: %.2f m" %self.leftfront_dist,
              "fd: %.2f m" %self.front_dist,
              "rfd: %.2f m" %self.rightfront_dist,
              "rd: %.2f m" %self.right_dist)
    
#.............................................................................................................


#Main.........................................................................................................
def main(args=None):
    rclpy.init(args=args)
    wallfollower = WallFollower()

    rclpy.spin(wallfollower)

    wallfollower.destroy_node
    rclpy.shutdown
#.............................................................................................................