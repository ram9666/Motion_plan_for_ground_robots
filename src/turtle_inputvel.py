#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
# node init
rospy.init_node('topic_publisher')
pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10) #topic publishing
rate = rospy.Rate(20)
move = Twist()

#giving linear vel in respective dire
rospy.loginfo("linear x= ")
move.linear.x = float(input())
rospy.loginfo("linear y= ")
move.linear.y = float(input())
rospy.loginfo("linear z= ")
move.linear.z = float(input())
#giving angular vel in respective
rospy.loginfo("angular x= ")
move.angular.x = float(input())
rospy.loginfo("angular y= ")
move.angular.y = float(input())
rospy.loginfo("angular z")
move.angular.z = float(input())
while True:
  pub.publish(move)#publishing
  rate.sleep()
