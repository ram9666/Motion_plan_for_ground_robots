#! /usr/bin/env python3

#importing req library
import rospy
from geometry_msgs.msg import Twist
import math
import time
from turtlesim.msg import Pose

pose = Pose()
def callback(msg):  #call back func for pose update
    global pose
    pose= msg
    
    
rospy.init_node('controller') #initializing node
pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, callback) #pose subscription

rate = rospy.Rate(100)
 
def goal(x1,y1):
    global distance
    #getting pose updates
    x2 = pose.x
    y2 = pose.y
    yaw = pose.theta
    move = Twist()
    distance = abs(math.sqrt(((x1-x2)**2)+ ((y1-y2)**2))) #dist btw pose and goal
    linear_k = 0.5 #constant
    linear_speed = linear_k * distance

    theta1 = math.atan2(y1-y2,x1-x2) #angle btw pose and goal
    angular_k = 2.0 #angulat constant
    angular_speed = abs(angular_k*(theta1- yaw))
    move.linear.x = linear_speed
    move.angular.z = angular_speed

    pub.publish(move) 
    rate.sleep()
    
#give input coordinates  
x1 = float(input("enter x: "))
y1 = float(input("enter y: "))

while True:
    goal(x1,y1) #fun_call
    if distance < 0.01 : #if dist btw turtle and goal is less 0.01 then it stops
        break