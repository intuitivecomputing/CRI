#!/usr/bin/env python

import rospy

from math import *
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import *
from arbotix_msgs.srv import SetSpeed
from arbotix_python.joints import *
from arbotix_python.servo_controller import *

rospy.init_node('demo')
pub1 = rospy.Publisher('/servo1_joint/command', Float64, queue_size=50)
pub2 = rospy.Publisher('/servo2_joint/command', Float64, queue_size=50)
pub3 = rospy.Publisher('/servo3_joint/command', Float64, queue_size=50)
pub4 = rospy.Publisher('/servo4_joint/command', Float64, queue_size=50)
pub5 = rospy.Publisher('/servo5_joint/command', Float64, queue_size=50)
pub6 = rospy.Publisher('/servo6_joint/command', Float64, queue_size=50)

servo1_min = -0.95
servo1_max = -0.25
servo3_min = -2.0
servo3_max = -1.3
servo4_min = 0.2
servo4_neutral = 0.4
servo4_max = 0.55
servo5_min = -0.3
servo5_max = 0.4
servo5_nod_min = -0.23
servo5_nod_max = -0.1
servo6_min = -1.0
servo6_neutral = -0.6
servo6_max = -0.4

global data
data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
publishers = [pub1, pub2, pub3, pub4, pub5, pub6]

def stateCb(msg):
    
    idx = msg.name.index('servo1_joint')
    data[0] = msg.position[idx]
    idx = msg.name.index('servo2_joint')
    data[1] = msg.position[idx]
    idx = msg.name.index('servo3_joint')
    data[2] = msg.position[idx]
    idx = msg.name.index('servo4_joint')
    data[3] = msg.position[idx]
    idx = msg.name.index('servo5_joint')
    data[4] = msg.position[idx]
    idx = msg.name.index('servo6_joint')
    data[5] = msg.position[idx]


def callBack(msg):
    message = msg.data
    print message
    if message == "eyelid":
        goal = [servo1_min, servo1_max]
        pubs = [pub1, pub2]
        move(goal,pubs,0)
    if message == "nod":
        goal = [servo5_nod_min, servo5_nod_max, servo5_nod_min, servo5_nod_max, servo5_nod_min, (servo5_min + servo5_max) / 2]
        pubs = [pub5]
        speed = SetSpeed_client(2, pubs)
        move(goal,pubs,4)
    if message == "shake":
        goal = [servo6_max, servo6_min, servo6_max, servo6_neutral]
        pubs = [pub6]
        speed = SetSpeed_client(1, pubs)
        move(goal,pubs,5)
    if message == "eye_h":
        goal = [servo3_max, servo3_min, (servo3_max + servo3_min) / 2]
        pubs = [pub3]
        speed = SetSpeed_client(1, pubs)
        move(goal,pubs,2)
    if message == "eye_v":
        goal = [servo4_max, servo4_min, servo4_neutral]
        pubs = [pub4]
        move(goal,pubs,3)
        
def move(goal,pubs,idx):
    for i in range(len(goal)):
        rospy.sleep(0.1)
        d = data[idx]
        count = 0
        while (abs(d - goal[i]) > 0.03 and count < 20000):
            count += 1
            d = data[idx]
            for pub in pubs:
                pub.publish(goal[i])
            



def SetSpeed_client(speed, pubs):
    servo_speed = ['/servo1_joint/set_speed', '/servo2_joint/set_speed', '/servo3_joint/set_speed', '/servo4_joint/set_speed', '/servo5_joint/set_speed', '/servo6_joint/set_speed']
    for pub in pubs:
        for item in range(6):
            if publishers[item] == pub:
                idx = item
                break
        rospy.wait_for_service(servo_speed[idx])
        try:
            set_speed = rospy.ServiceProxy(servo_speed[idx],SetSpeed)
            s = set_speed(speed)
            return s
        except rospy.ServiceException, e:
            print ("Service call failed")
        
rospy.Subscriber('/gesture', String, callBack)     
rospy.Subscriber('joint_states', JointState, stateCb)

rospy.spin()
        
