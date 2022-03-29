#!/usr/bin/env python3
#license removed for brevity

#from ssl import VERIFY_ALLOW_PROXY_CERTS
import rospy
import math
from math import pow,atan2,sqrt,asin
import time
import numpy as np
from  nav_msgs.msg import OccupancyGrid,Odometry,Path
from geometry_msgs.msg import PoseStamped
import rospy
from std_msgs.msg import Float64

def vel_sub(msg):
    global vel1
    vel1 = msg.data
    print("vel1 is",vel1)
rospy.init_node("veli",anonymous=True)
i =0
while(True):
   vel = rospy.Subscriber('/joint1_vel_controller/command',Float64,vel_sub)
