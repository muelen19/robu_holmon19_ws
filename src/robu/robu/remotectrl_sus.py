#Exercise: Mein Erster Node - Fernsteuerung des Roboters
#Group: 1
#Class: 4BHME
#Date: 13.10.2022

from pydoc import doc
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import msvcrt


msg = """
Excercise:  x
Group:      2
Class:      BHME19
Date:       today
"""

e = """
Communications Failed
"""
#75...K links
#77...M rechts
#72...H vorwärts
#80...P rückwärtss
#27...Escape
key_ctrl = [75, 77, 72, 80]

def get_key():
    try:
        return msvcrt.getch().decode('cp1252') #.decode('utf-8')
    except:
        return msvcrt.getch().decode('utf-8')


def main():
    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('remotectrl')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    vel = Twist()
    MAX_LIN_VEL = 0.22 #m/s
    MIN_LIN_VEL = -0.22    #m/s
    MAX_ANG_VEL = 2.82 #rad/s
    MIN_ANG_VEL = -2.82 #rad/s

    LIN_VEL_STEP_SIZE = 0.01 #m/s
    ANG_VEL_STEP_SIZE = 0.1  #rad/s
    key_null_entered = False
    try:
        print(msg)
        #add Code here
        while(1):
            key = get_key()
            #print("ASCII Zeichen", key, "Code: ", ord(key))

            if ord(key) == 0x00:
                key_null_entered = True
            elif ord(key) == 27: #Escape wurde gedrueckt -> Roboter sollte stehen bleiben
                print("Escape")
                vel.linear.x = 0.0
                vel.linear.y = 0.0
                vel.linear.z = 0.0
                vel.angular.z = 0.0
            
            #Gerade
            elif ord(key) == 103: #g drücken um wieder gerade aus zu fahren
                print("Gerade")
                vel.angular.z = 0.0

            #Stop
            elif ord(key) == 3: #STRG+C
                break

            #Links, Rechts, Vorwärts, Rückwärts
            elif key_null_entered == True and ord(key) in key_ctrl:
                if key_ctrl[0] == ord(key):     #Links
                    print("Links")
                    if vel.angular.z >= (MIN_ANG_VEL - ANG_VEL_STEP_SIZE):
                        vel.angular.z += ANG_VEL_STEP_SIZE
                    else:
                        vel.angular.z = MIN_ANG_VEL
                elif key_ctrl[1] == ord(key):   #Rechts
                    print("Rechts")                  
                    if vel.angular.z <= (MAX_ANG_VEL + ANG_VEL_STEP_SIZE):
                        vel.angular.z -= ANG_VEL_STEP_SIZE
                    else:
                        vel.angular.z = MAX_ANG_VEL
                elif key_ctrl[2] == ord(key):   #Schneller
                    print("Vorwaerts")                                       
                    if vel.linear.x <= (MAX_LIN_VEL - LIN_VEL_STEP_SIZE):
                        vel.linear.x +=  LIN_VEL_STEP_SIZE
                    else:
                        vel.linear.x = MAX_LIN_VEL

                elif key_ctrl[3] == ord(key):   #Langsamer
                    print("Rückwaerts")
                    if vel.linear.x >= (MIN_LIN_VEL + LIN_VEL_STEP_SIZE):
                        vel.linear.x -= LIN_VEL_STEP_SIZE
                    else:
                        vel.linear.x = MIN_LIN_VEL
                key_null_entered = False
            print(vel)
            pub.publish(vel)
                
    except Exception as e:
        print(e)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
