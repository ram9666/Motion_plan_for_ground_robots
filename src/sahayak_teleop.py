#!/usr/bin/env python3
#license removed for brevity

import rospy
from std_msgs.msg import Float64
def joint_vel(a,b,c,d):
    joint1 = rospy.Publisher('/joint1_vel_controller/command',Float64,queue_size=10)
    joint2 = rospy.Publisher('/joint2_vel_controller/command',Float64,queue_size=10)
    joint3 = rospy.Publisher('/joint3_vel_controller/command',Float64,queue_size=10)
    joint4 = rospy.Publisher('/joint4_vel_controller/command',Float64,queue_size=10)
    #publishing vel
    joint1.publish(a)
    joint2.publish(b)
    joint3.publish(c)
    joint4.publish(d)

rospy.init_node("move",anonymous=True)
rate = rospy.Rate(10)

while True:
    key = input("enter a key \n")
    if key == 'a':
         print("moving left ")
         joint_vel(20,20,20,20) #giving vel to each wheel
    if key == 'd':
       print('moving right ')
       joint_vel(-20,-20,-20,-20)
    if key == 'w':
        print("moving forward")
        joint_vel(20,-10,20,-10)
    if key == 's':
        print("moving backward ")
        joint_vel(-20,20,-20,20)
    if key == 'q':
        print("staying stopped")
        joint_vel(0,0,0,0)
    rate.sleep()

