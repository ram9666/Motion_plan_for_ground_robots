#!/usr/bin/env python3
import rospy 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import time
import numpy as np

import cv2 
import numpy as np
import math
import priority_dict

color_map = cv2.imread('map.pgm')
#map3 = map13[1450:2000,1510:1800]
map1= cv2.cvtColor(color_map,cv2.COLOR_BGR2GRAY)
#map3 = map3[1450:2000,1510:1800]
ret,map1 = cv2.threshold(map1, 250,255,cv2.THRESH_BINARY)


def heuristic_dist(state_key, goal_key):
    x1,y1 = state_key
    x2,y2 = goal_key
    dist = float(((x2-x1)**2 + (y2-y1)**2 )**0.5)
    return dist

def nearby_node(v, map1):
    neighbour1 = []
    x,y = v
    for i in range(-1,2):
        for j in range(-1,2):
            if i == 0 and  j ==0:
                continue
            distance = float(((i**2+j**2)**0.5))
            pixel = map1[x+i, y+j]
            if pixel == 255:
                neighbour1.append((x+i, y+j,distance))
            elif pixel == 0:
                distance = math.inf
                neighbour1.append((x+i, y+j,distance))         
    return neighbour1

def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    while (key != origin_key):
        key = predecessors[key]
        path.insert(0, key)
    return path

def a_star_search(origin_key, goal_key, img):
    open_queue = priority_dict.priority_dict({})
    closed_dict = {}
    predecessors = {}
    predecessors[origin_key]= 0
    costs = {}
    costs[origin_key] = 0.0
    open_queue[origin_key] =heuristic_dist(origin_key,goal_key)

    goal_found = False
    while (open_queue):
        u,length= open_queue.pop_smallest()
        uCost = costs[u]
        if u == goal_key:
            goal_found = True
            break
        for edge in nearby_node(u,img):
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
        raise ValueError("Goal not found in search.")
    return get_path(origin_key, goal_key, predecessors)


start = (170,210)
goal = (750,300)
path = a_star_search(start, goal, map1)

def plot(img,path):
    for i in path:
        x,y = i
        img[x,y]=150
    cv2.imshow('map',img)
    cv2.waitKey(0)
plot(map1,path)