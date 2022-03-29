#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, termios, tty, os, time
#function to take inputs continously as long as button hold without pressing enter button
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)	    
    try:
       tty.setraw(sys.stdin.fileno())
       ch = sys.stdin.read(1)
    except:
         pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# node initialization
rospy.init_node('topic_publisher')
pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10) #topic publishing
rate = rospy.Rate(100)

def move1(inp_key):
    move = Twist()
    if inp_key == "w" or inp_key == "W":
        move.linear.x = 1 #linera vel of 1 in x dir
    elif inp_key == "s" or inp_key == "S":
        move.linear.x = -1 #lin_vel of -1 in -x dir
    elif inp_key == "a" or inp_key == "A":
        move.angular.z = 1 #ang_vel in z anticlock
    elif inp_key == "d" or inp_key == "D":
        move.angular.z=-1  
     
    pub.publish(move)
    rate.sleep()

while True:
    inp_key = getch()  #calling func
    print("enter_key: ",inp_key) 
    move1(inp_key)
    if inp_key == "q":
        break
    
