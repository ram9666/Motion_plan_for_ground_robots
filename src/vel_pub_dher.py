#!/usr/bin/env python3
#license removed for brevity

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from math import pow,atan2,sqrt,asin
import math
from nav_msgs.msg import Path


def callback(pose_msg):
    global x,y,xq,yq,zq,wq,pose
    x= pose_msg.pose.pose.position.x
    y= pose_msg.pose.pose.position.x

    # u= pose_msg.twist.twist.linear.x
    # v= pose_msg.twist.twist.linear.y
    # c= sqrt(u**2+v**2)

    # w= pose_msg.twist.twist.angular.z
    # print(pose_msg)
    xq = pose_msg.pose.pose.orientation.x
    yq = pose_msg.pose.pose.orientation.y
    zq = pose_msg.pose.pose.orientation.z
    wq = pose_msg.pose.pose.orientation.w
    
    pose = (x,y)

#def track(path_msg):
 #       global route
  #      route = path_msg.poses


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def joint_name(number):
    name= '/joint'+ str(number)+'_vel_controller/command'
    return name


if __name__ == '__main__':
    global x,y,xq,yq,zq,wq

        
    rospy.init_node('vel_publisher', anonymous=True)
    path = rospy.wait_for_message("/path_finder", Path)
    
    joints=[]
    pub=[]
    total_joints=4
    rate= rospy.Rate(10)

    pose_msg = rospy.wait_for_message('/ground_truth/state',Odometry)

    xq,yq,zq,wq = pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w
    
    pos = rospy.Subscriber("/ground_truth/state",Odometry, callback)
    
    for i in range(total_joints):
        joints.append(joint_name(i+1))
        pub.append(rospy.Publisher(joints[i], Float64 , queue_size=10))



    loc = path.poses
    way = list()
    
    for pose in loc:
        # print(type(pose))
        xi = pose.pose.position.x
        yi = pose.pose.position.y
        way.append((xi,yi))
            

    R= 0.08
    L= 0.41
    w_int= 0
    old_e = 0
    E = 0
    w=0
    goal = way[-1]
    dx = goal[0]
    dy = goal[1]
    
    dist = sqrt((dx-x)**2 + (dy-y)**2)
    
    while way:
        
        roll_x, pitch_y, yaw_z = euler_from_quaternion(xq, yq, zq, wq)
        
        point = way.pop(0)
        des_x , des_y = point

        K_v=6
        K_d=4
        K_i= 2

        error= sqrt(pow(des_x-x,2)+pow(des_y-y,2)) #Proportional term
        e_dot= error - old_e #Differential term
        E = E + error #Integral term
        V= K_v*dist + K_d*e_dot - K_i*E
        old_e = error

        print(error)

        theta_star= atan2(des_y-y,des_x-x)
        k_t= 200
        k_d= 150
        k_i= 50


        theta= yaw_z
        w_star= theta_star-theta
        w_dot= w_star-w
        w_int= w_int + w_star 
        W=k_t*w_star+ k_d*w_dot + k_i*w_int

        w= w_star

        Vl = V/R - (W*L)/(2*R)
        Vr = V/R + (W*L)/(2*R)

        pub[0].publish(Vl)
        pub[1].publish(-Vr)
        pub[2].publish(Vl)
        pub[3].publish(-Vr)
#        rate.sleep()


    rospy.spin()
