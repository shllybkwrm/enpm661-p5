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

