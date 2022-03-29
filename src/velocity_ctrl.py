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


def joint_vel(a,b,c,d):
    joint1 = rospy.Publisher('/joint1_vel_controller/command',Float64,queue_size=10)
    joint2 = rospy.Publisher('/joint2_vel_controller/command',Float64,queue_size=10)
    joint3 = rospy.Publisher('/joint3_vel_controller/command',Float64,queue_size=10)
    joint4 = rospy.Publisher('/joint4_vel_controller/command',Float64,queue_size=10)
    joint1.publish(a)
    joint2.publish(b)
    joint3.publish(c)
    joint4.publish(d)

rospy.init_node("move",anonymous=True)
rate = rospy.Rate(10)

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

occupancy_map=OccupancyGrid()
path=Path()
goal1=PoseStamped()
start1 = Odometry()
def vel_sub(msg):
    global velo
    velo= msg.data
    print("vel1 is:",velo)


def map_call(msg):
    global occupancy_map
    occupancy_map = msg

def start_call(msg):
    global start1
    start1 = msg

def goal_call(msg):
    global goal1
    goal1=msg
    print(goal1)

def map1(occupancy_map):
    map_arr=np.array(occupancy_map.data)
    #print(map_arr)
    map_arr= np.reshape(map_arr,(3328,3328))
    #print(map_arr)
    #
    return map_arr

def heuristic_dist(state_key, goal_key):
    x1,y1 = state_key
    x2,y2 = goal_key
    dist = float(((x2-x1)**2 + (y2-y1)**2 )**0.5)
    return dist

def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
    #print(path)
    return path

def nearby_node(v, map12):
    neighbour1 = []
    x,y= v
    #print(x,y)
    for i in range(-1,2):
        for j in range(-1,2):
            if (i == 0) and  (j ==0):
                continue
            distance = float(((i**2+j**2)**0.5))
            pixel = map12[x+i,y+j]
            #print(pixel)
            #print(pixel)
            if pixel == 0 :
                #print("free",pixel)
                neighbour1.append((x+i, y+j,distance))
            elif pixel==100:
                 #print("occupied",pixel)
                 dist = math.inf
                 neighbour1.append((x+i, y+j,dist)) 
            elif pixel==-1:
                dist = math.inf
                neighbour1.append((x+i, y+j,dist)) 
            #print(neighbour1)
    #print("yes  ",neighbour1)      
    return neighbour1


def a_star_search(origin_key, goal_key, map1):
    open_queue = {}
    closed_dict = {}
    predecessors = {}
    predecessors[origin_key]= 0
    costs = {}
    costs[origin_key] = 0.0
    open_queue[origin_key] =heuristic_dist(origin_key,goal_key)

    goal_found = False
    while (open_queue):
        u= min(open_queue,key=open_queue.get)
        #print("u is ",u)
        ucost=open_queue.pop(u)
        uCost = costs[u]
        #print("ucost is ",uCost)
        if u == goal_key:
            goal_found = True
            break
        for edge in nearby_node(u,map1):
            v = (edge[0],edge[1])
            uvCost = (edge[2])
            if v in closed_dict:
                continue

            if v in open_queue:
                if uCost + uvCost + heuristic_dist(v, goal_key) < open_queue[v]:
                    open_queue[v] = uCost + uvCost +heuristic_dist(v, goal_key)
                    costs[v]= uCost+uvCost
                    predecessors[v] = u
            else:
                open_queue[v] = uCost +uvCost + heuristic_dist(v, goal_key)
                costs[v] = uCost +uvCost
                predecessors[v] = u
        closed_dict[u] = uCost

    if not goal_found:
        print("Goal not found in search.")
    return get_path(origin_key, goal_key, predecessors)
#global q_x,q_y,q_z,q_w
def callback(pose_msg):
    global x,y, q_x,q_y,q_z,q_w,start1
    start1 = pose_msg
    x= pose_msg.pose.pose.position.x
    y= pose_msg.pose.pose.position.y
    #print("is here",x,y)
    q_x = pose_msg.pose.pose.orientation.x
    #print("qx is ",q_x)
    q_y = pose_msg.pose.pose.orientation.y
    q_z = pose_msg.pose.pose.orientation.z
    q_w = pose_msg.pose.pose.orientation.w    


if __name__=='__main__':
   #rospy.init_node("shortest_path",anonymous=True)
   path_p=rospy.Publisher("/path_finder",Path,queue_size=10)
   rospy.wait_for_message("/map",OccupancyGrid)
   map_sub=rospy.Subscriber("/map",OccupancyGrid,map_call)
   
   while not rospy.is_shutdown():
      rospy.wait_for_message("/ground_truth/state",Odometry)
      rospy.Subscriber("/ground_truth/state",Odometry,callback)
      start_v=start1.pose.pose.position
      start = (int((start_v.y+50.010000)/0.030000),int((start_v.x+50.010000)/0.030000))
      #start=(1667, 1667)
      print("start is ",start)

      rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
      goal_sub=rospy.Subscriber("/move_base_simple/goal",PoseStamped,goal_call)
      goal_v=goal1.pose.position
      goal = (int((goal_v.y+50.010000)/0.030000),int((goal_v.x+50.010000)/0.030000))
      #goal = (1666, 1567)
      print("goal is ",goal)

      map_2d=map1(occupancy_map) #getting 2d map
      shorter_path=a_star_search(start,goal,map_2d)
      path2=Path()
      path2.header.frame_id="map" #assigning frame  id
      co_od = []

      for i in shorter_path:
         x=i[1]*0.030000-50.010000
         y=i[0]*0.030000-50.010000
         points = x , y
         co_od.append(points)
         pos=PoseStamped()
         pos.pose.position.x=x
         pos.pose.position.y=y
         path2.poses.append(pos) #appending list to path
         path_p.publish(path2)

      w_int= 0
      old_e = 0
      E = 0
      w=0
      R= 0.08
      L= 0.41
      veloc = []
      (dx,dy) = goal
      dist = sqrt((dx-x)**2 + (dy-y)**2)
      for i in co_od:
        req_x,req_y = i
        #print(q_x)
        #print("is ",i)
        roll_x, pitch_y, yaw_z = euler_from_quaternion(q_x,q_y,q_z,q_w)
        theta_star= atan2(req_y-y,req_x-x)
        theta_net= theta_star - yaw_z
        K_v=0.0003
        K_d=0.0015
        K_i= 0.00010
        error= sqrt(pow(req_x-x,2)+pow(req_y-y,2)) #Proportional term
        e_dot= error - old_e #Differential term
        E = E + error #Integral term
        V= K_v*dist + K_d*e_dot + K_i*E
        old_e = error
        #print(error)
        theta_star= atan2(req_y-y,req_x-x)
        k_t= 0.20
        k_d= 0.15
        k_i= 0.05
 
        theta= yaw_z
        w_star= theta_star-theta
        w_dot= w_star-w
        w_int= w_int + w_star 
        W=k_t*w_star+ k_d*w_dot + k_i*w_int
        
        w= w_star
        #V  = 20
       # W = 20

        Vl = V/R - (W*L)/(2*R)
        Vr = V/R + (W*L)/(2*R)
        #l = Vl/1000
        #Vr = Vr/1000
        #print("1st is " ,Vl,"2nd is ",Vr)
        veloc.append((Vl,Vr))
        #joint_vel(0,0,0,0)
        #joint_vel(Vl,Vr,Vl,Vr)
        #rate.sleep()
        #if i == co_od[-1]:
            #joint_vel(0,0,0,0)
    #except rospy.ROSInterruptException:
     #   pass
     #rospy.spin()
     # veloc.append((0,0))
      for j in veloc:
            # print(j)
             Vl,Vr = j
             print("vel1_pub :",Vl)
             joint_vel(Vl,-Vr,Vl,-Vr)
             #rospy.Subscriber('/joint1_vel_controller/command',Float64,vel_sub)

     # vel = rospy.Subscriber('/joint1_vel_controller/command',Float64,vel_sub)
      #joint_vel(0,0,0,0)
#
                

