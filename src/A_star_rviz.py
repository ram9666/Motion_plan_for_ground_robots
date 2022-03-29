#!/usr/bin/env python3

import rospy
import math
import time
import numpy as np
from  nav_msgs.msg import OccupancyGrid,Odometry,Path
from geometry_msgs.msg import PoseStamped

occupancy_map=OccupancyGrid()
path=Path()
goal1=PoseStamped()
start1 = Odometry()

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
    print(path)
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


if __name__=='__main__':
   rospy.init_node("shortest_path",anonymous=True)
   path_p=rospy.Publisher("/path_finder",Path,queue_size=10)
   rospy.wait_for_message("/map",OccupancyGrid)
   map_sub=rospy.Subscriber("/map",OccupancyGrid,map_call)
   
   while not rospy.is_shutdown():
      rospy.wait_for_message("/ground_truth/state",Odometry)
      rospy.Subscriber("/ground_truth/state",Odometry,start_call)
      start_v=start1.pose.pose.position
      start = (int((start_v.y+50.010000)/0.030000),int((start_v.x+50.010000)/0.030000))
      #start=(1667, 1667)
      print("start is ",start)

      rospy.wait_for_message("/move_base_simple/goal",PoseStamped)
      goal_sub=rospy.Subscriber("/move_base_simple/goal",PoseStamped,goal_call)
      goal_v=goal1.pose.position
      goal = (int((goal_v.y+50.010000)/0.030000),int((goal_v.x+50.010000)/0.030000))
      print("goal is ",goal)

      map_2d=map1(occupancy_map) #getting 2d map
      shorter_path=a_star_search(start,goal,map_2d)
      path2=Path()
      path2.header.frame_id="map" #assigning frame  id

      for i in shorter_path:
         x=i[1]*0.030000-50.010000
         y=i[0]*0.030000-50.010000
         pos=PoseStamped()
         pos.pose.position.x=x
         pos.pose.position.y=y
         path2.poses.append(pos) #appending list to path
         path_p.publish(path2)