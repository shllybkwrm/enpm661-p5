#!/usr/bin/env python
import random, math
from math import sqrt,cos,sin,atan2
import math as m
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches
from matplotlib.patches import Ellipse
import cv2
from ObstacleMap import ObsMap

NUMNODES = 3000	#5000

# Generating neighbor nodes
def generateNeighborNodes(node, rpm_1, rpm_2):		   # UL = velocity left, UR = velocity right
	Xi = float(node[0])
	Yi = float(node[1])
	ThetaDeg = float(node[2])
	neighbors = []
	RPM1 = rpm_1
	RPM2 = rpm_2
	actions = [[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]
	#actions = [[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1],[RPM1,-RPM2],[RPM2,-RPM1],[-RPM1,RPM2],[-RPM2,RPM1],[0,-RPM1],[-RPM1,0],[-RPM1,-RPM1],[0,-RPM2],[-RPM2,0],[-RPM2,-RPM2],[-RPM1,-RPM2],[-RPM2,-RPM1]] 
 	
	for action in actions:
		X1, Y1, Theta, curve_len = calculate_coord(Xi,Yi,ThetaDeg,action[0],action[1]) # (0,0,45) hypothetical start configuration
		neighbors.append((round(X1,3), round(Y1,3), round(Theta),action[0],action[1], round(curve_len,3)))
	return neighbors
	
def dist(p1,p2):
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

# calculating distance between 2 coordinate points 	
def calc_distance(x1, y1, x2, y2):
	dist = m.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	return dist

# Calculating coordinates of new node
def calculate_coord(Xi, Yi, ThetaDeg, UL, UR):
	t = 0
	r = 0.038			# turtlebot tire radius (mm)
	L = 0.354		   # turtlebot distance between wheels	(mm)
	dt = 0.1			 # reasonable dt assigned 
	Xn=Xi
	Yn=Yi
	ThetaRad = 3.14 * ThetaDeg / 180				# Theta angle in radians
	curve_len = 0
	while t<1:	# 1
		t = t + dt
		Xs = Xn
		Ys = Yn
		Xn += (r /2)* (UL + UR) * m.cos(ThetaRad) * dt
		Xn = round(Xn,2)
		Yn += (r /2 )* (UL + UR) * m.sin(ThetaRad) * dt
		Yn = round(Yn,2)
		ThetaRad += (r /L) * (UR - UL) * dt
		curve_len += calc_distance(Xs, Ys, Xn, Yn)
	ThetaDeg = 180 * (ThetaRad) / 3.14
	ThetaDeg = round(ThetaDeg)
	if ThetaDeg >= 360:
		ThetaDeg = (ThetaDeg - 360)
	if ThetaDeg <= -360:
		ThetaDeg = (ThetaDeg + 360)
	return Xn, Yn, ThetaDeg, curve_len

# Visited nodes discrete matrix to check for duplicates
def visited_nodes_duplicate():
	visited_node_duplicate = {}
	for x in np.arange(-50,51,1)/10:
		for y in np.arange(-50,51,1)/10:
			for theta in np.arange(0,120,10)/10:
				visited_node_duplicate[x,y,theta]=0
	return visited_node_duplicate

# Node Cost discrete matrix
def exploredNodesCost_discrete():
	explrdNdCost_discrete = {}
	for x in np.arange(-50,51,1)/10:
		for y in np.arange(-50,51,1)/10:
			for theta in np.arange(0,120,10)/10:
				explrdNdCost_discrete[x,y,theta]=0
	return explrdNdCost_discrete
	
# Defining Obstacle Space using Half Plane Equations while also adding obstacle clearance and robot radius
def rigid_robot_obstacle_space(x,y,clearance,radius): #clearance = 0.025,radius = 0.127
	obstacle = False
	coord_offset_x= 0
	coord_offset_y = 0
	
	offset_dist = clearance + radius
	
	if ((x - coord_offset_x)**2 + (y - coord_offset_y)**2 - (1 + offset_dist)**2) <= 0:	# Circle in center of map
		obstacle = True

	if ((x - (coord_offset_x + 2))**2 + (y - (coord_offset_y + 3))**2 - (1 + offset_dist)**2) <= 0:	# Circle on top corner of map
		obstacle = True

	if ((x - (coord_offset_x - 2))**2 + (y - (coord_offset_y - 3))**2 - (1 + offset_dist)**2) <= 0:	# Circle on left lower corner of map
		obstacle = True

	if ((x - (coord_offset_x + 2))**2 + (y - (coord_offset_y - 3))**2 - (1 + offset_dist)**2) <= 0:	# Circle on right lower corner of map
		obstacle = True
			
	if ( x + 4.75 + offset_dist >= 0) and (x + 3.25	 - offset_dist<= 0) and (y - 0.75 - offset_dist <= 0) and (y + 0.75 + offset_dist >= 0):	   #  x1,x2,y1 UP, y2 DWN  --> square on left side of map 
		obstacle = True

	if ( x - 3.25 + offset_dist >= 0) and (x - 4.75 - offset_dist <= 0) and (y - 0.75 - offset_dist <= 0) and (y + 0.75 + offset_dist >= 0):	   # square on right side of map 
		obstacle = True
			
	if (x + 2.75 + offset_dist >= 0) and (x + 1.25 - offset_dist <= 0) and (y - 2.25 + offset_dist >= 0) and (y - 3.75 - offset_dist <= 0):	   # square on top left side of map 
		obstacle = True
			
	return obstacle	

# Checking for node to see if it has valid coordinates
def isNewNodeValid(curr_nd, clearance, radius):
		valid = False
		if curr_nd[0] < -5 or curr_nd[1] < -5 :	  
			valid = True
		if curr_nd[0] > 5 or curr_nd[1] > 5 :
			valid = True
		if rigid_robot_obstacle_space(curr_nd[0],curr_nd[1],clearance,radius) == True :
			valid = True
		return valid

# Check if node is duplicate 
def is_node_duplicate(curr_nd,vis_nd_dupl):
	duplicate = False
	if curr_nd in vis_nd_dupl:
		if vis_nd_dupl[curr_nd] == 1:
			duplicate = True
		else:
			vis_nd_dupl[curr_nd] = 1
	return duplicate

# Rounding nodes to one decimal point
def roundToNearestPoint1(node):
	if node[2] < 0:
		theta_disc	 = node[2] + 360
	else:
		theta_disc = node[2]
	theta_disc = int(theta_disc/30)
	rnd_node = (round(node[0],1),round(node[1],1),theta_disc)
	return rnd_node

# Calculate heuristic distance between 2 nodes
def euclidean_dist(n_node, g_node):
	distX = g_node[0] - n_node[0]
	distY = g_node[1] - n_node[1]
	euclDist = m.sqrt((distX)**2 + (distY)**2)
	return euclDist
	
def rrt_star(start_nd, goal_nd, goal_rad, clearance, radius, rpm1, rpm2):
	nodes = []
	child_parent = {}
	nodes.append(start_nd)											# Initialize tree with start node (x_start) : Step 1
	child_parent[start_nd] = None
	nodesCost = {}
	#nodesCost = exploredNodesCost_discrete()  	##   # Contains list of explored nodes cost
	nodesCost[start_node] = 0					##
	i = 1
	goal_achieved = False
	ng_cnt = 0
	#while i < NUMNODES:
	while not goal_achieved:											# Looping through number of desired iterations : Step 2
		rand_x  = random.randrange(-50,50,1)/10					# random sample (x_samp) : Step 3
		rand_y  = random.randrange(-50,50,1)/10
		rand = (rand_x,rand_y)
		if isNewNodeValid(rand, clearance, radius):
			continue
		#print("random number generated = ",rand)
		nearest_node = nodes[0]											
		for p in nodes:											# Finding nearest node (x_nearest) : Step 4
			if dist(p,rand) < dist(nearest_node,rand):
				nearest_node = p
		neighbors_list = generateNeighborNodes(nearest_node, rpm1, rpm2)
		node_min = neighbors_list[0] 
		noNeighbor = True
		for node in neighbors_list:
			if isNewNodeValid(node, clearance, radius) :
				continue
			if dist(node,rand) < dist(node_min, rand):
				noNeighbor = False
				node_min = node
		if noNeighbor:
			continue
		#print("nearest node",nearest_node)
		#print(neighbors_list)
		#print("neighbor with shortest distance to sample = ", node_min)
		#newnode = step_from_to(nearest_node,rand)							# Determining new node (x_new) in direction of x_sampled using nearest node : Step 5				
		# Collision detection should be added here				# If node is within obstacle space or out of boundaries then discard : Step 6
		#if isNewNodeValid(newnode, clearance, radius) :
		#	continue
		#print(node_min)
		nodes.append(node_min)									# If node not colliding with obstacles or outside boundaries then add to List : Step 7
		child_parent[node_min] = nearest_node
		nodesCost[node_min] = node_min[5] + nodesCost[nearest_node]
		#print("nearest node cost = ",nodesCost[nearest_node])
		#print("node_min cost = ",node_min[5])
		#print("Nodes cost = ",nodesCost[node_min])
		plot_curved_line(nearest_node, node_min, "orange")

		## NEED TO CHECK IF GENERATED NEW NODE IS VALID
		XnearList = []
		for p in nodes:											# Finding nearest node (x_nearest) : Step 4
			dist_near_new = dist(p,node_min)
			if dist_near_new < (3*dist(nearest_node,node_min)) and p!= nearest_node and p!= node_min:
				XnearList.append([p,dist_near_new])				
		
		if XnearList:
			XnearMinCost = XnearList[0][1]
			XnMinNearest_node = XnearList[0][0]
			for node in XnearList:
				if node[1] < XnearMinCost:
					XnearMinCost = node[1]
					XnMinNearest_node = node[0]
			#print("List of nodes close to new node = ",XnearList)
			#print("minimum node = ",XnMinNearest_node)
			#print("minimum cost", XnearMinCost)
			nearest_neighbors_list = generateNeighborNodes(XnMinNearest_node, rpm1, rpm2)	
			nearest_node_min = neighbors_list[0] 
			noNeighbor = True
			for node in nearest_neighbors_list:
				if isNewNodeValid(node, clearance, radius) :
					continue
				if dist(node,rand) < dist(nearest_node_min, rand):
					noNeighbor = False
					nearest_node_min = node
			if noNeighbor:
				continue
			nodes.append(nearest_node_min)									# If node not colliding with obstacles or outside boundaries then add to List : Step 7
			child_parent[nearest_node_min] = XnMinNearest_node
			nodesCost[nearest_node_min] = nearest_node_min[5] + nodesCost[XnMinNearest_node]
			plot_curved_line(XnMinNearest_node, nearest_node_min, "orange")

		# If node is within goal threshold						# Break up from while loop is node is within radius threshold of goal : Step 8
		if dist(node_min,goal_nd) <= goal_rad:
			goal_achieved = True
			print("goal achieved")
			plot_curved_line(nearest_node, node_min, "blue")
			print("number of nodes = ", i)
			return node_min, nodes, child_parent
		i = i + 1

# Implementing backtracking function to retrieve optimal path from visited nodes   
def backtrackingStartGoalPath(start,goal_thd,explored_path):
	pathlist = []
	goalpath = goal_thd
	pathlist.append(goal_thd)
	while goalpath != start:
		pathlist.append(explored_path[goalpath])
		goalpath = explored_path[goalpath]
	pathlist.reverse()
	return pathlist	

# Plotting curve line between 2 coordinate points
def plot_curved_line(nodePar,nodeCh,linecolor):
	t = 0
	r = 0.038			# turtlebot tire radius (mm)
	L = 0.354		   # turtlebot distance between wheels	(mm)
	dt = 0.1			 # reasonable dt assigned 
	Xn = nodePar[0]
	Yn = nodePar[1]
	ThetaRad = 3.14 * nodePar[2] / 180
	UL = nodeCh[3]
	UR = nodeCh[4]

	while t<1:	# 1
		t = t + dt
		Xs = Xn
		Ys = Yn
		Xn += (r /2)* (UL + UR) * m.cos(ThetaRad) * dt
		Xn = round(Xn,2)
		Yn += (r /2 )* (UL + UR) * m.sin(ThetaRad) * dt
		Yn = round(Yn,2)
		ThetaRad += (r /L) * (UR - UL) * dt
		plt.plot([Xs, Xn], [Ys, Yn], color=linecolor, linewidth=0.5)
	ThetaDeg = 180 * (ThetaRad) / 3.14
	ThetaDeg = round(ThetaDeg)
	if ThetaDeg >= 360:
		ThetaDeg = (ThetaDeg - 360)
	if ThetaDeg <= -360:
		ThetaDeg = (ThetaDeg + 360)
		
	mapImg = bufImage()
	if cv2.waitKey(1) == ord('q'):	# 10
		exit()
	cv2.imshow('RRT Star Algorithm', mapImg)
	#return Xn, Yn, ThetaDeg

def plot_lines(pt1,pt2,linecolor):
	x1=pt1[0]
	y1=pt1[1]
	x2 = pt2[0]
	y2 = pt2[1]
	deltaX = x2 - x1
	deltaY = y2 - y1
	line = plt.Arrow(x1,y1, deltaX, deltaY, color=linecolor, width = 0.05)
	mapImg = bufImage()
	if cv2.waitKey(1) == ord('q'):	# 10
		exit()
	cv2.imshow('RRT Star Algorithm', mapImg)
	return line 


	
	
