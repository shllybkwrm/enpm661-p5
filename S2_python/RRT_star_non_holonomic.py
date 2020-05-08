# Scenario 2: RRT star program for non-holonomic constraints

# The dubin curves generated in this program are used from the work done by Andrew Walker for dubin curves in c code  
# The reference citations are included in the project paper accompanying this code. 
# Also a separate file including the MIT license from the original Author is included in our project folder. 

# This program runs RRT algorithm on a map of a hot cell room with obstacles.
# It uses differential drive constraints and dubins path curves

#!/usr/bin/env python
import random, math
from math import sqrt,cos,sin,atan2
from math import pi, sin, acos, tan, floor
import math as m
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches
from matplotlib.patches import Ellipse
import cv2
from enum import Enum
from prj5_map import ObsMap
from ReadExposureDict import ReadDict
import time

NN_Radius = 30
NUMBER_OF_SAMPLES = 400

# Defining Obstacle Space using Half Plane Equations while also adding obstacle clearance and robot radius
def rigid_robot_obstacle_space(x,y,clearance,radius): #clearance = 0.025,radius = 0.127
	obstacle = False
	coord_offset_x= 0
	coord_offset_y = 0
	
	offset_dist = clearance + radius
	rad_obs = 30

	if ((x - (coord_offset_x + 150))**2 + (y - (coord_offset_y + 120))**2 - (1 + rad_obs)**2) <= 0:	# Source 1
		obstacle = True

	if ((x - (coord_offset_x + 366))**2 + (y - (coord_offset_y + 76))**2 - (1 + rad_obs)**2) <= 0:	# Source 2
		obstacle = True

	if ((x - (coord_offset_x + 620))**2 + (y - (coord_offset_y + 120))**2 - (1 + rad_obs)**2) <= 0:	# Source 3
		obstacle = True		

	if ((x - (coord_offset_x + 200))**2 + (y - (coord_offset_y + 100))**2 - (20 + offset_dist)**2) <= 0:	# Cask1_location
		obstacle = True

	if ((x - (coord_offset_x + 400))**2 + (y - (coord_offset_y + 100))**2 - (20 + offset_dist)**2) <= 0:	# Cask2_location
		obstacle = True

	if ((x - (coord_offset_x + 600))**2 + (y - (coord_offset_y + 100))**2 - (20 + offset_dist)**2) <= 0:	# Cask1_location
		obstacle = True

	return obstacle	

#Calculate and return new node coordinates  	
def distance_from_to(p1,p2):
	if euclidean_dist(p1,p2) < NN_Radius:
		theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
		theta = round(m.degrees(theta))
		if theta >= 360:
			theta = (theta - 360)
		if theta <= -360:
			theta = (theta + 360)
		return round(p2[0]),round(p2[1]),theta
	else:
		theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
		theta = round(m.degrees(theta))
		if theta >= 360:
			theta = (theta - 360)
		if theta <= -360:
			theta = (theta + 360)
		return round(p1[0] + NN_Radius*cos(theta)), round(p1[1] + NN_Radius*sin(theta)), theta

# Checking for node to see if it has valid coordinates
def isNewNodeValid(curr_nd, clearance, radius):
		valid = False
		if curr_nd[0] <= (0 + 15) or curr_nd[1] <= (0 + 15) :	  
			valid = True
		if curr_nd[0] >= (732 - 15) or curr_nd[1] >= (152 - 15) :
			valid = True
		if rigid_robot_obstacle_space(curr_nd[0],curr_nd[1],clearance,radius) == True :
			valid = True
		return valid
		
# Checking for node to see if it has valid coordinates
def isDubinValid(curr_nd, clearance, radius):
		valid = False
		if curr_nd[0] <= (0 + 15) or curr_nd[1] <= (0 + 15) :	  
			valid = True
		if curr_nd[0]/10 >= (732 - 15) or curr_nd[1]/10 >= (152 - 15) :
			valid = True
		if rigid_robot_obstacle_space(curr_nd[0],curr_nd[1],clearance,radius) == True :
			valid = True
		return valid

# calculating distance between 2 coordinate points 	
def calc_distance(x1, y1, x2, y2):
	dist = m.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	return dist

#Round to nearest 5 multiple
def nearest_five(x_in,y_in):
	x_out = int(x_in / 5) * 5
	y_out = int(y_in / 5) * 5
	return x_out, y_out
	
# Calculate heuristic distance between 2 nodes
def euclidean_dist(n_node, g_node):
	distX = g_node[0] - n_node[0]
	distY = g_node[1] - n_node[1]
	euclDist = m.sqrt((distX)**2 + (distY)**2)
	return euclDist

# Implementing RRT star algorithm
def rrt_star(start_nd, goal_nd, goal_rad, clearance, radius, t_exposure):
	goal_list = []
	goal_node_cost_list = {}
	nodes = []
	child_parent = {}
	nodes.append(start_nd)					# Initialize tree with start node
	child_parent[start_nd] = None
	i = 1
	nodesCost = {}
	nodesCost[start_node] = 0
	goal_achieved = False
	while i < NUMBER_OF_SAMPLES:				# Looping through number of desired iterations 
		rand_x  = random.randrange(0,732,1)				# random sample
		rand_y  = random.randrange(0,152,1)
		rand = (rand_x,rand_y)
		if isNewNodeValid(rand, clearance, radius):
			continue
		nn = nodes[0]											
		for p in nodes:					# Finding nearest node 
			if euclidean_dist(p,rand) < euclidean_dist(nn,rand):
				nn = p
		node_min = distance_from_to(nn,rand)
		if isNewNodeValid(node_min, clearance, radius):
			continue
		pts = [nn, node_min]
		turning_rad1, dubin_segment1, dubin_type1 = compute_dubin_curve(pts[0], pts[1], 20, 35) 
		path, newNdCost = compute_trajectory(nn, turning_rad1, dubin_segment1, dubin_type1, 1)
		obs_flag = False
		for node in path:
			obs_flag = isDubinValid(node,clearance,radius)
			if obs_flag == True:
				break
		if obs_flag == False:
			nodes.append(node_min)			# Add new node to tree if not colliding 
			child_parent[node_min] = nn
			nodesCost[node_min] = newNdCost/10 + nodesCost[nn]
			plot_curved_line(nn, node_min, "orange")	
			XnearList = []
			for p in nodes:							# Finding nearest node 
				pts = [p, node_min]
				turning_rad2, dubin_segment2, dubin_type2 = compute_dubin_curve(pts[0], pts[1], 20, 35) 
				path, dist_near_ = compute_trajectory(p, turning_rad2, dubin_segment2, dubin_type2, 1)
				dist_near_new = dist_near_/10
				if dist_near_new <= 1.5*NN_Radius and p!= nn and p!= node_min:
					XnearList.append([p,dist_near_new])
			if XnearList:
				XnearMinCost = XnearList[0][1]
				XnMinNearest_node = XnearList[0][0]
				for node in XnearList:
					if node[1] < XnearMinCost:
						XnearMinCost = node[1]
						XnMinNearest_node = node[0]
				pts = [XnMinNearest_node, node_min]
				turning_rad3, dubin_segment3, dubin_type3 = compute_dubin_curve(pts[0], pts[1], 20, 35) 
				path, newNearNdCost = compute_trajectory(XnMinNearest_node, turning_rad3, dubin_segment3, dubin_type3, 1)
				x_min = nn
				c_min = nodesCost[node_min]  
				cost_xnear = nodesCost[XnMinNearest_node] + newNearNdCost/10
				for node in path:
					obs_flag = isDubinValid(node,clearance,radius)
					if obs_flag == True:
						break
				if obs_flag == False:
					if cost_xnear < c_min:
						x_min = XnMinNearest_node
						c_min = cost_xnear
					# If node not colliding with obstacles or outside boundaries then add to List : Step 7
					child_parent[node_min] = x_min
					nodesCost[node_min] = c_min
					plot_curved_line(XnMinNearest_node, node_min, "green")		
		# If node is within goal threshold then add it to goal list						
		if euclidean_dist(node_min,goal_nd) <= goal_rad:
			#goal_achieved = True
			goal_node_cost_list[node_min] =  nodesCost[node_min]
			goal_list.append(node_min)
			print("goal achieved")
			plot_curved_line(nn, node_min, "black")
			print("number of nodes to reach goal = ", i)
			
		i = i + 1
		print("sample ",i)
	return node_min, nodes, child_parent, goal_list, goal_node_cost_list, nodesCost

# Implementing backtracking function to retrieve optimal path from visited nodes   
def backtrackingStartGoalPath(start,goal_thd,explored_path, nd_cost,x_y_exp,t_exposure):
	pathlist = []
	gl_cost_list = []
	goalpath = goal_thd
	pathlist.append(goal_thd)
	print("RRT-star path Total Cost = ",nd_cost[goalpath])	
	while goalpath != start:
		pathlist.append(explored_path[goalpath])
		gl_cost_list.append(nd_cost[goalpath])
		goalpath = explored_path[goalpath]
	pathlist.reverse()
	gl_cost_list.reverse()
	i = 1
	for node in gl_cost_list:
		x_rd5,y_rd5 = nearest_five(pathlist[i-1][0],pathlist[i-1][1])
		x_y_exp += float(t_exposure[(x_rd5,y_rd5)]/3600)   # converting exposure/hr to exposure/seconds
		print(x_y_exp)
		print("Edge cost up to node ",i, " =",node)
		i += 1
	return pathlist,x_y_exp	

# Plotting line between nodes
def plot_curved_line(nodePar,nodeCh,linecolor):
	pts = [nodePar, nodeCh]
	turning_rad, dubin_segment, dubin_type = compute_dubin_curve(pts[0], pts[1], 20, 35) 
	path, length = compute_trajectory(nodePar, turning_rad, dubin_segment, dubin_type, 1)
	plt.plot(pts[0][0],pts[0][1],'.',color='orange')
	plt.plot(pts[1][0],pts[1][1],'.',color='orange')
	plt.plot(path[:,0]/10,path[:,1]/10,color=linecolor, linewidth=1)
	mapImg = bufImage()
	if cv2.waitKey(1) == ord('q'):	
		exit()
	cv2.imshow('RRT Algorithm', mapImg)

# Plotting optimal path provided by rrts star 
def plot_rrt_path(rrtPath):
	rrtStarLen = len(rrtPath)-1
	print("rrt-star Len = ",rrtStarLen)
	i = 0
	rrtPathNode = None
	while i < rrtStarLen:
		rrtPathNode = (rrtPath[i],rrtPath[i+1])
		plot_curved_line(rrtPathNode[0],rrtPathNode[1], "red")
		mapImg = bufImage()		
		i = i+1
	
# Buffering image 
def bufImage():
	Obs_space.fig.canvas.draw()
	mapImg = np.frombuffer(Obs_space.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(Obs_space.fig.canvas.get_width_height()[::-1] + (3,))
	mapImg = cv2.cvtColor(mapImg,cv2.COLOR_RGB2BGR)
	return mapImg
