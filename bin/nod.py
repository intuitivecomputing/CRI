#!/usr/bin/env python

import rospy

from math import *

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import *
from arbotix_msgs.srv import SetSpeed
from arbotix_python.joints import *
from arbotix_python.servo_controller import *

rospy.init_node('nod')
pub = rospy.Publisher('/servo5_joint/command', Float64, queue_size=5)

servo1_min = -1.0
servo1_max = -0.2
servo3_min = -1.8
servo3_max = -1.66
servo4_min = -0.5
servo4_neutral = 0.2
servo4_max = 0.4
servo5_min = -0.3
servo5_max = 0.2
servo5_nod_min = -0.25
servo5_nod_max = -0.05
servo6_min = -1.2
servo6_neutral = -0.55
servo6_max = -0.1

global count

global finish

global connection

count = 0

finish = False

connection = False

pub2 = rospy.Publisher('/servo3_joint/command', Float64, queue_size=5)

movement = [servo5_nod_min,servo5_nod_max,servo5_nod_min]

i = 0

goal = movement[i]

def stateCb(msg):
    rospy.sleep(1.)
    speed = SetSpeed_client(1)
    pub.publish(0.2)
    rospy.sleep(3.)
    pub.publish(-0.2)
    rospy.sleep(3.)

    global count
    global finish
    global data
    global smooth
    count += 1
    idx = msg.name.index('servo5_joint')
    smooth = True
    if count > 2:
        if abs(data - msg.position[idx]) < 0.0001:
            smooth = False
    data = msg.position[idx]
    speed = SetSpeed_client(0.1)
    if abs(data - goal) > 0.03:
        Move(goal)
    else:
        nod()
        count = 0
    return True

def callBack(msg):
    global connection
    if (msg.data):
        print("Hello, Maki!!!!!")
        while not rospy.is_shutdown():
            rospy.Subscriber('joint_states', JointState, stateCb)
              
    
def Move(goal):
    pub.publish(goal)

rospy.Subscriber('/connect', Bool, callBack)


def SetSpeed_client(speed):
    rospy.wait_for_service('/servo5_joint'+'/set_speed')
    try:
        set_speed = rospy.ServiceProxy('/servo5_joint'+'/set_speed',SetSpeed)
        s = set_speed(speed)
        return s
    except rospy.ServiceException, e:
        print ("Service call failed")
        
def nod():
    global i
    global goal
    i += 1
    if i < len(movement):
        goal = movement[i]
        print ("phase",i,goal)
        

rospy.spin()
        
