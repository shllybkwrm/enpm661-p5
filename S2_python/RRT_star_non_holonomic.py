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

# Dubins LSL calculation	
def LSL(alpha, beta, d): 
    p2_lsl = 2 + d**2 - (2*cos(alpha-beta)) + (2*d*(sin(alpha) - sin(beta)))
    if p2_lsl>=0:
        t = (-alpha + atan2((cos(beta)- cos(alpha)),(d + sin(alpha) - sin(beta)))) % (pi*2)
        p = sqrt(p2_lsl)
        q = (beta - atan2((cos(beta)- cos(alpha)),(d + sin(alpha) - sin(beta)))) % (pi*2)
    else:
        p = q = t = 100000
    return t, p, q

#Dubins RSR calculation
def RSR(alpha, beta, d): 
    p2_rsr = 2 + d**2 - (2*cos(alpha-beta)) + 2*d*(sin(beta) - sin(alpha))
    if p2_rsr>=0:
        t = (alpha - atan2((cos(alpha) - cos(beta)),(d - sin(alpha) + sin(beta)))) % (pi*2)
        p = sqrt(p2_rsr)
        q = (-beta + atan2((cos(alpha) - cos(beta)),(d - sin(alpha) + sin(beta)))) % (pi*2)
    else:
        p = q = t = 100000
    return t, p, q

#Dubins RSL calculation
def RSL(alpha,beta,d): 
    p2_rsl = -2 + d**2 + 2*cos(alpha - beta) - 2*d*(sin(alpha) + sin(beta))
    if p2_rsl>=0:
        p = sqrt(p2_rsl)
        t = (alpha - atan2((cos(alpha)+cos(beta)),(d-sin(alpha)-sin(beta))) + atan2(2,p)) % (pi*2)
        q = (beta - atan2((cos(alpha)+cos(beta)),(d-sin(alpha)-sin(beta))) + atan2(2,p)) % (pi*2)
    else:
        p = q = t = 100000
    return t, p, q

#Dubins LSR calculation
def LSR(alpha, beta, d): 
    p2_lsr = d**2 - 2 + 2*cos(alpha-beta) + 2*d*(sin(alpha) + sin(beta))
    if p2_lsr>=0:
        p = sqrt(p2_lsr)
        t = ((atan2((-1*cos(alpha)-cos(beta)),(d+sin(alpha)+sin(beta)))+atan2(2,p))-alpha) % (pi*2)
        q = ((atan2((-1*cos(alpha)-cos(beta)),(d+sin(alpha)+sin(beta)))+atan2(2,p))-beta) % (pi*2)
    else:
        p = q = t = 100000
    return t, p, q

#Dubins RLR calculation
def RLR(alpha, beta, d):
    p_rlr = (6 - d**2 + 2*cos(alpha-beta) + 2*d*(sin(alpha)-sin(beta)))/8
    if(abs(p_rlr)<=1):
        p = (2*pi - acos(p_rlr)) % (pi*2)
        t = (alpha - atan2((cos(alpha)-cos(beta)), d-sin(alpha)+sin(beta)) + p/2 % (pi*2)) % (pi*2)
        q = (alpha - beta - t + (p % (pi*2))) % (pi*2)
    else:
        p = q = t = 100000
    return t, p, q

#Dubins LRL calculation
def LRL(alpha, beta, d):
    p_lrl = ((6 - d**2 + 2*cos(alpha-beta) + 2*d*(sin(beta)-sin(alpha)))/8)
    if(abs(p_lrl)<=1):
        p = (2*pi - acos(p_lrl)) % (pi*2)
        t = (-alpha - atan2((cos(alpha)-cos(beta)), d+sin(alpha)-sin(beta)) + p/2) % (pi*2)
        q = ((beta % (pi*2))-alpha-t +(p % (pi*2))) % (pi*2)
    else:
        p = q = t = 100000
    return t, p, q

# Dubins curve coefficients calculation
def compute_dubin_curve(pt1,pt2,vel,angle_lim):
	pt1_angle = convert_angle(90 - pt1[2])
	pt2_angle = convert_angle(90 - pt2[2])
	turning_rad = (vel**2)/(9.8*tan(angle_lim*pi/180))
	dx = pt2[0]*10 - pt1[0]*10
	dy = pt2[1]*10 - pt1[1]*10
	dist = sqrt(dx**2 + dy**2)
	d = dist/turning_rad
	theta = atan2(dy,dx) % (pi*2)
	alpha = (pt1_angle - theta) % (pi*2)
	beta = (pt2_angle - theta) % (pi*2)
	dubin_seg = {}
	t1, p1, q1 = LSL(alpha,beta,d)
	len_LSL = t1 + p1 + q1
	if len_LSL >= 0:
		dubin_seg[len_LSL] = (0,[t1,p1,q1])
	t2, p2, q2 = LSR(alpha,beta,d)
	len_LSR = t2 + p2 + q2
	if len_LSR >= 0:
		dubin_seg[len_LSR] = (1,[t2,p2,q2])
	t3, p3, q3 = RSL(alpha,beta,d)
	len_RSL = t3 + p3 + q3
	if len_RSL >= 0:
		dubin_seg[len_RSL] = (2,[t3,p3,q3])
	t4, p4, q4 = RSR(alpha,beta,d)
	len_RSR = t4 + p4 + q4
	if len_RSR >= 0:
		dubin_seg[len_RSR] = (3,[t4,p4,q4])	
	t5, p5, q5 = RLR(alpha,beta,d)
	len_RLR = t5 + p5 + q5
	if len_RLR >= 0:
		dubin_seg[len_RLR] = (4,[t5,p5,q5])		
	t6, p6, q6 = LRL(alpha,beta,d)
	len_LRL = t6 + p6 + q6
	if len_LRL >= 0:
		dubin_seg[len_LRL] = (5,[t6,p6,q6])
	min_cost_path = min(dubin_seg)
	dubin_type = dubin_seg[min_cost_path][0]
	dubin_segment = dubin_seg[min_cost_path][1]
	return turning_rad, dubin_segment, dubin_type

# Angle conversion to radians
def convert_angle(theta):
	if theta >= 360:
		theta = (theta - 360)
	if theta <= -360:
		theta = (theta + 360)
	theta_rad = theta*pi/180
	return theta_rad

# Computing Dubins trajectory
def compute_trajectory(p1, turning_rad, dubin_segment, dubin_type,step):
    length = floor(((dubin_segment[0]+dubin_segment[1]+dubin_segment[2])*turning_rad)/step)
    length = int(length)
    dub_path = np.zeros((length,3))
    i = j = 0
    while i < length:
        dub_path[j] = compute_path(p1,turning_rad,dubin_segment,dubin_type,i)
        i += step
        j+=1
    return dub_path, length

#Computing dubin path for curves
def compute_path(p1,turn_rad,dubin_segment,dubin_type, i):
    xp = i/turn_rad
    ang = convert_angle(90 - p1[2])
    pStart = np.array([0,0,ang])
    matTp = np.array([[1,2,1],[1,2,3],[3,2,1],[3,2,3],[3,1,3],[1,3,1]])
    seg_t = matTp[dubin_type][:]
    sp1 = compute_segment(dubin_segment[0],pStart,seg_t[0])
    sp2 = compute_segment(dubin_segment[1],sp1,seg_t[1])
    if(xp<dubin_segment[0]):
        endP = compute_segment(xp,pStart,seg_t[0])
    elif(xp<(dubin_segment[0]+dubin_segment[1])):
        endP = compute_segment(xp-dubin_segment[0],sp1,seg_t[1])
    else:
        endP = compute_segment(xp-dubin_segment[0]-dubin_segment[1], sp2, seg_t[2])
    endP[0] = endP[0]*turn_rad+p1[0]*10
    endP[1] = endP[1]*turn_rad+p1[1]*10
    endP[2] = endP[2] % (2*pi)
    return endP

# Computing Dubin path segments
def compute_segment(xp, segm, seg_t):
    segEnd = np.array([0.0,0.0,0.0])
    if(seg_t == 1):
        segEnd[0] = sin(segm[2]+xp)-sin(segm[2])
        segEnd[1] = -cos(segm[2]+xp)+cos(segm[2])
        segEnd[2] = xp
    elif(seg_t == 2):
        segEnd[0] = cos(segm[2])*xp
        segEnd[1] = sin(segm[2])*xp
        segEnd[2] = 0
    elif(seg_t == 3):
        segEnd[0] = -sin(segm[2]-xp)+sin(segm[2])
        segEnd[1] = cos(segm[2]-xp)-cos(segm[2])
        segEnd[2] = -xp
    segEnd[0] = segm[0] + segEnd[0]
    segEnd[1] = segm[1] + segEnd[1]
    segEnd[2] = segm[2] + segEnd[2]
    return segEnd
