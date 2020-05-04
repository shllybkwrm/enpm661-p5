# -*- coding: utf-8 -*-
"""
ENPM661 Project 5, Spring 2020
Shelly Bagchi

Use this file to read in the exposure values for Scenario 2
and plot them on a heat map
"""

import ast
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.colors import LogNorm


def ReadDict():
    f = open("Total_Exposure.txt", "r")
    contents = f.read()
    exposure_dict = ast.literal_eval(contents)
    #exposure_dict = eval(contents)
    f.close()
    
    return exposure_dict


def plotObstacles(ax):
    Source1_location = [150,120]
    Source2_location = [366,76]
    Source3_location = [620,120]
    Cask1_location = [200,100]
    Cask2_location = [400,100]
    Cask3_location = [600,100]
    Cask_rad = 20
    
    def makeRect(xcoord,ycoord,length, width):
        x=np.linspace((xcoord-length/2),(xcoord+length/2),length+1,dtype=int)
        y=np.linspace((ycoord+width/2),(ycoord-width/2),width+1,dtype=int)
        x_1,y_1=np.meshgrid(x,y, indexing='xy')
        points = np.array(list(zip(x_1.flatten(),y_1.flatten())))
        vertices = [((xcoord-length/2), (ycoord-width/2)), ((xcoord-length/2), (ycoord+width/2)), ((xcoord+length/2), (ycoord+width/2)), ((xcoord+length/2), (ycoord-width/2)), (0, 0)]
        return points, vertices
    
    #################### ROBOT COORDINATE
    robot_x_coord=30  #LOCATION OF THE EQUIPMENT ENTRY DOOR WHERE ROBOT WILL START
    robot_y_coord=30  #LOCATION OF THE EQUIPMENT ENTRY DOOR WHERE ROBOT WILL START
    robot_height=6
    robot_breadth=6
    Robot_points,rvertices=makeRect(robot_x_coord,robot_y_coord,robot_breadth,robot_height)
    ########### PLOTTING THE ROBOT ################
    rcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
    robot = Path(rvertices,rcodes)
    robotpatch = PathPatch(robot, facecolor='green', edgecolor='green')
    #################### PLOTTING SOURCE 1
    circle=Path.circle(Source1_location, radius=1)
    source1patch=PathPatch(circle, facecolor='red', edgecolor='red')
    #################### PLOTTING SOURCE 2
    circle=Path.circle(Source2_location, radius=1)
    source2patch=PathPatch(circle, facecolor='red', edgecolor='red')
    #################### PLOTTING SOURCE 3
    circle=Path.circle(Source3_location, radius=1)
    source3patch=PathPatch(circle, facecolor='red', edgecolor='red')
    
    #################### PLOTTING DRY STORAGE CASK 1
    circle=Path.circle(Cask1_location, radius=Cask_rad)
    sourcedrycast1=PathPatch(circle, facecolor='white', edgecolor='black')
    #################### PLOTTING DRY STORAGE CASK 2
    circle=Path.circle(Cask2_location, radius=Cask_rad)
    sourcedrycast2=PathPatch(circle, facecolor='white', edgecolor='black')
    #################### PLOTTING DRY STORAGE CASK 3
    circle=Path.circle(Cask3_location, radius=Cask_rad)
    sourcedrycast3=PathPatch(circle, facecolor='white', edgecolor='black')
    #################### TOOL BOX COORDINATE 
    TB_x_coord=400  # LOCATION OF TOOL BOX CENTER 
    TB_y_coord=40  # LOCATION OF TOOL BOX CENTER 
    TB_length= 200
    TB_height= 10
    TB_points, TBvertices=makeRect(TB_x_coord,TB_y_coord,TB_length,TB_height)
    TB = Path(TBvertices,rcodes)
    TBpatch = PathPatch(TB, facecolor='purple', edgecolor='purple')
    
    
    ax.add_patch(robotpatch)
    ax.add_patch(TBpatch)
    ax.add_patch(source1patch)
    ax.add_patch(source2patch)
    ax.add_patch(source3patch)
    ax.add_patch(sourcedrycast1)
    ax.add_patch(sourcedrycast2)
    ax.add_patch(sourcedrycast3)
    return ax


def plotHeatMap(fig,ax, exposure_dict):
    
    room_length = 732 # cm
    room_width = 152 # cm
    
    # Include discretization factor here (needs to match Exposure_function.py)
    grid = 1 #cm
    Xgrid = range(0, room_length, grid)
    Ygrid = range(0, room_width, grid)
    
    Total_Exposure = np.zeros((room_width, room_length))
    
    for i in Xgrid:
        for j in Ygrid:
            Total_Exposure[j,i] = exposure_dict[(i,j)]
    
    
    im = ax.imshow(Total_Exposure, norm=LogNorm())
    ax.set_title('Total exposure rate in log scale (scenario 2)')
    cb = fig.colorbar(im)
    cb.set_label("Exposure rate (R/hr)")
    
    return fig,ax


def plotMap(exposure_dict=None):
    
    room_length = 732 # cm
    room_width = 152 # cm
    
     
    fig,ax = plt.subplots()

    if exposure_dict!=None:
        fig,ax = plotHeatMap(fig,ax, exposure_dict)
    ax = plotObstacles(ax)
    
    ax.autoscale_view()
    plt.xlim(0,room_length)
    plt.ylim(0,room_width)
    plt.gca().set_aspect('equal', adjustable='box')
    
    return fig,ax


if __name__ == "__main__":
    exposure_dict = ReadDict()
    print(exposure_dict)
    
    fig,ax = plotMap(exposure_dict)
    # Note:  To plot without heatmap, send in no parameters:
    fig1,ax1 = plotMap()
    # You can add the heatmap afterwards:
    fig1,ax1 = plotHeatMap(fig1,ax1, exposure_dict)
    
    plt.show()
    
    