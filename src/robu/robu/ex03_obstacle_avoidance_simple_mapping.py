import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import time

class ObastacleAvoidanceSimple(Node):
    def __init__(self, regional_angle_deg = 30, normal_lin_vel=0.1, trans_lin_vel=-0.02, trans_ang_vel=1.0):
        super().__init__('ObstacleAvoidanceSimple')

        """************************************************************
        ** Initialise variables
        ************************************************************"""

        """
        :param regional_angle    : The angle on which each region extends
        """

        self.scan = None
        self.cmd_vel_raw = None

        self.REGIONAL_ANGLE_DEG = regional_angle_deg
        self.OBSTACLE_DIST = 0.3
        
        self.vel_obj = Twist()

        self.NORMAL_LIN_VEL = normal_lin_vel
        self.TRANS_LIN_VEL  = trans_lin_vel
        self.TRANS_ANG_VEL  = trans_ang_vel

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)
        
        
        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.timer_callback)

        self.get_logger().info("Obstacle avoidance simple node has been initialised.")

    def scan_callback(self, msg):
        self.scan = msg
        #print(self.scan)

    def cmd_vel_raw_callback(self, msg):
        self.cmd_vel_raw = msg

    def timer_callback(self):
        steer, ang_vel = self.obstacle_avoidance()
        #print("steer: ", steer, "ang_vel: ", ang_vel)
        self.steer(steer, ang_vel)

    def obstacle_avoidance(self):
        if self.scan == None:
            return False, 0.0

        region_size = int(360 / self.REGIONAL_ANGLE_DEG + 0.5)
        region_distance_size = int(len(self.scan.ranges) / region_size) #int(((self.scan.angle_min + math.radians(self.REGIONAL_ANGLE_DEG)) / self.scan.angle_increment) + 0.5)


        #0      0
        #1      11
        #2      1
        #3      10
        #4      2
        #5      9
        #6      3
        #7      8
        #8      4
        #9      7
        #10     5
        #11     6

        region_order = [0] * region_size
        for i in range(int((region_size)/2)):
            region_order[2*i] = i
            region_order[2*i+1] = region_size-i-1
        region_order[-1] = int((region_size)/2)
        
        region_distances = region_size * [0]
        #print("region_size:", region_size, "region_distance_size:", region_distance_size)
        #print("region_order", region_order)
        
        #front region
        region_begin = -int(region_distance_size/2)
        region_end = region_begin + region_distance_size
        distances = self.scan.ranges[region_begin : ] + self.scan.ranges[ : region_end]
        #print("distances: ", distances)
        region_distances[0] = [x for x in distances if x <= self.OBSTACLE_DIST and x != 'inf']

        for i in range(1,region_size):
            region_begin = region_end
            region_end = region_begin + region_distance_size
            distances = self.scan.ranges[region_begin : region_end]
            region_distances[i] = [x for x in distances if x <= self.OBSTACLE_DIST and x != 'inf']
            #print("region: ", i, "region_begin:", region_begin, "region_end:", region_end, "scan-size:", len(self.scan.ranges))

        maxima = 0
        region = 0
        for i in region_order:
            """
            if len(region_distances[i]):
                print("region: ", i, "max-distance: ", max(region_distances[i]))
            else:
                print("region: ", i, "max-distance: >", self.OBSTACLE_DIST)
            """
            if not len(region_distances[i]):    #no obstacles within OBSTACLE_DIST found!
                maxima = self.OBSTACLE_DIST
                region = i
                break
            elif max(region_distances[i]) > maxima:
                maxima = max(region_distances[i])
                region = i

        if region == 0:
            ang_vel = 0.0
        elif region > int((region_size)/2): #turn left
            ang_vel = -self.TRANS_ANG_VEL
        else:
            ang_vel = self.TRANS_ANG_VEL
        
        #print("region: ", region, "ang_vel: ", ang_vel)
        
        return (region != 0), ang_vel

    def steer(self, steer=False, ang_vel=0.0):
        '''
        :param steer  : Whether to avoid and obstacle or keep on going straigt
        :param ang_vel: The angular velocity of the robot
        '''
        try:
            if not steer:
                if self.cmd_vel_raw != None:
                    self.vel_obj.linear.x = self.cmd_vel_raw.linear.x
                    self.vel_obj.angular.z = self.cmd_vel_raw.angular.z
                else:
                    self.vel_obj.linear.x = 0.0 #self.NORMAL_LIN_VEL
                    self.vel_obj.angular.z = 0.0 #ang_vel
            else:
                self.vel_obj.linear.x = self.TRANS_LIN_VEL
                self.vel_obj.angular.z = ang_vel

            self.vel_obj.linear.y  = 0.0
            self.vel_obj.linear.z  = 0.0
            self.vel_obj.angular.x = 0.0
            self.vel_obj.angular.y = 0.0
        
        except:
            self.vel_obj.linear.x = 0.0
            self.vel_obj.linear.y  = 0.0
            self.vel_obj.linear.z  = 0.0
            self.vel_obj.angular.x = 0.0
            self.vel_obj.angular.y = 0.0
            self.vel_obj.angular.z = 0.0
        
        self.cmd_vel_pub.publish(self.vel_obj)

"""
    def __del__(self):
        self.vel_obj.linear.x = 0.0
        self.vel_obj.linear.y  = 0.0
        self.vel_obj.linear.z  = 0.0
        self.vel_obj.angular.x = 0.0
        self.vel_obj.angular.y = 0.0
        self.vel_obj.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel_obj)
        time.sleep(1)
"""

def main(args=None):
    rclpy.init(args=args)

    obstacleavoidance_node = ObastacleAvoidanceSimple()

    #rclpy.on_shutdown(shutdownhook)  # add this line to your main program 
    rclpy.spin(obstacleavoidance_node)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacleavoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()