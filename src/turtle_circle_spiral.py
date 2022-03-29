#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher') #node initiaolizing
pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10) #pub topic
#rate = rospy.Rate(1)
move = Twist()

move.angular.z= 1 #angular vel
#move.angular.z = 4
n = 0.5 
while n >=0:
    rate = rospy.Rate(1)
    move.linear.x = n
    pub.publish(move)
    rate.sleep()
    n= n+0.05 #increaseing linear by some value in loop
