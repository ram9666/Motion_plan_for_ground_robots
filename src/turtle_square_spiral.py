#! /usr/bin/env python3

import rosgraph
import rospy
from geometry_msgs.msg import Twist
from math import pi
import time
from turtlesim.msg import Pose
 
def callback(msg):
    global pose
    pose= msg

rospy.init_node('squre')
pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, callback)
pose = Pose()
move  = Twist()

PI = 3.1415926535897

def rotate():
    
      # Receiveing the user
    speed = float(100)
    angle = float(90)
    clockwise = 0

    #Converting from angles to radians
    angular_speed = float(speed*2*PI/360)
    relative_angle = float(angle*2*PI/360)
  
  # Checking if our movement is CW or CCW
    if clockwise:   
        move.angular.z = -abs(angular_speed)
    else:
       move.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle <=relative_angle):
        pub.publish(move)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)   #Forcing our robot to stop
    move.angular.z = 0
    pub.publish(move)
    
d = 1
while (d<=20):
    x1 = round(pose.x,1)
    move.linear.x = d
    pub.publish(move)
    if x1 == 6.6+1:
        move.linear.x =0 
        pub.publish(move)
        rotate()
        d = d+1
    if x1 == 6.6+2:
        move.linear.x =0 
        pub.publish(move)
        rotate()
        d = d+1
        
      
print(x1)
#move.linear.x = 0
#pub.publish(move)
#rotate()

    
        