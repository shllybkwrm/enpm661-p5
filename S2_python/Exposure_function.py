# -*- coding: utf-8 -*-
"""
ENPM661 Project 5, Spring 2020
Omololu Makinde, Shelly Bagchi

Use this file to calculate the exposure values for Scenario 2
and plot them on a heat map.
Also contains ROS outputs
"""

"""
Westinghouse Hot Cell

The high-level cell was initially designed and constructed
to accommodate the size and radiological conditions
associated with a full-sized Westinghouse commercial plant
fuel assembly. As such, this particular cell is relatively large
(24 feet long x 5 feet deep x 12 feet high) and is constructed
with extremely thick high-density poured concrete walls
(minimum wall thickness of 27 inches) for additional
radiological shielding.

The high-level cell has four remote handling stations and
multiple remote cameras with live video feed viewing monitors.
A heavy-duty manipulator with a hoist capability of 1,000 pounds...
"""


#from pylab import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.colors import LogNorm
#import itertools
import sys


room_length = 732 # cm
room_width = 152 # cm
# Reordered left to right
Source1_location = [150,120]
Source2_location = [366,76]
Source3_location = [620,120]
Cask1_location = [200,100]
Cask2_location = [400,100]
Cask3_location = [600,100]
Cask_rad = 20


def py_ROScoord(pycoord):
    roscoordx=np.abs((pycoord[0]/100)-3.66)
    roscoordy=np.abs((pycoord[1]/100)-0.76)
    ans=[roscoordx,roscoordy]
    print("ROS COORDINATES",pycoord, ans)
    return ans

RSource1_location = py_ROScoord(Source1_location)
RSource2_location = py_ROScoord(Source2_location)
RSource3_location = py_ROScoord(Source3_location)
RCask1_location = py_ROScoord(Cask1_location)
RCask2_location = py_ROScoord(Cask2_location)
RCask3_location = py_ROScoord(Cask3_location)
start= [30,30]
Rstart=py_ROScoord(start)

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
sourcedrycast1=PathPatch(circle, facecolor='white', edgecolor='none')
#################### PLOTTING DRY STORAGE CASK 2
circle=Path.circle(Cask2_location, radius=Cask_rad)
sourcedrycast2=PathPatch(circle, facecolor='white', edgecolor='none')
#################### PLOTTING DRY STORAGE CASK 3
circle=Path.circle(Cask3_location, radius=Cask_rad)
sourcedrycast3=PathPatch(circle, facecolor='white', edgecolor='none')
#################### TOOL BOX COORDINATE 
TB_x_coord=400  # LOCATION OF TOOL BOX CENTER 
TB_y_coord=40  # LOCATION OF TOOL BOX CENTER 
TB_length= 200
TB_height= 10
TB_points, TBvertices=makeRect(TB_x_coord,TB_y_coord,TB_length,TB_height)
TB = Path(TBvertices,rcodes)
TBpatch = PathPatch(TB, facecolor='purple', edgecolor='purple')

###### PLOTTING #####################
##fig, ax = plt.subplots()
##ax.add_patch(robotpatch)
##ax.add_patch(TBpatch)
##ax.add_patch(source1patch)
##ax.add_patch(source2patch)
##ax.add_patch(source3patch)
##ax.add_patch(sourcedrycast1)
##ax.add_patch(sourcedrycast2)
##ax.add_patch(sourcedrycast3)
##ax.set_title("Westinghouse Hot Cell")
##
##ax.autoscale_view()
##plt.xlim(0,732)
##plt.ylim(0,152)


#################### Defining the map #########################
### Moved these to top ###
# room_length = 732 # cm
# room_width = 152 # cm
# Source1_location = [400,60]
# Source2_location = [150,120]
# Source3_location = [620,120]
x=np.linspace(-room_length/2,room_length/2,room_length+1)
y=np.linspace(-room_width/2,room_width/2,room_width+1)
X,Y = np.meshgrid(x,y, indexing='xy')
a=np.array(((0,X),(0,Y)))
R=np.linalg.norm(a) #The 2 norm of each point gives the distance from the center of the room
points = np.array(list(zip(X.flatten(),Y.flatten())))


####################### Helper funcs for half-planes determined by cask-source perpendicular lines
# https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
#  Uses cross-product/2D determinant
def isLeft(a, b, c):  # a,b two points on ends of line, c is new point to check
    return ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) > 0

def findLinePts(Cask_loc, Source_loc):
    # perpendicular slope = -1/m
    m = (-1)/((Source_loc[1] - Cask_loc[1]) / (Source_loc[0] - Cask_loc[0]))
    b = Cask_loc[1] - m*Cask_loc[0]  # y-intercept: b=y-mx using cask center
    x0 = (0-b)/m  # x-intercept: x=(y-b)/m
    xmax = (room_width-b)/m
    print(m, b, x0, xmax)
    a = (x0,0)
    b = (xmax,room_width)
    return a,b

### Find parameters of lines bisecting casks
# Cask 1
print('\n',"Cask1-Source1 line params")
a_c1s1, b_c1s1 = findLinePts(Cask1_location, Source1_location)
ans = isLeft(a_c1s1, b_c1s1, (300,100))
print(ans)

print("Cask1-Source2 line params")
a_c1s2, b_c1s2 = findLinePts(Cask1_location, Source2_location)
ans = isLeft(a_c1s2, b_c1s2, (300,100))
print(ans)

print("Cask1-Source3 line params")
a_c1s3, b_c1s3 = findLinePts(Cask1_location, Source3_location)
ans = isLeft(a_c1s3, b_c1s3, (300,100))
print(ans, '\n')


# Cask 2 
print("Cask2-Source1 line params")
a_c2s1, b_c2s1 = findLinePts(Cask2_location, Source1_location)
ans = isLeft(a_c2s1, b_c2s1, (300,100))
print(ans)

print("Cask2-Source2 line params")
a_c2s2, b_c2s2 = findLinePts(Cask2_location, Source2_location)
ans = isLeft(a_c2s2, b_c2s2, (300,100))
print(ans)

print("Cask2-Source3 line params")
a_c2s3, b_c2s3 = findLinePts(Cask2_location, Source3_location)
ans = isLeft(a_c2s3, b_c2s3, (300,100))
print(ans, '\n')


# Cask 3 
print("Cask3-Source1 line params")
a_c3s1, b_c3s1 = findLinePts(Cask3_location, Source1_location)
ans = isLeft(a_c3s1, b_c3s1, (300,100))
print(ans)

print("Cask3-Source2 line params")
a_c3s2, b_c3s2 = findLinePts(Cask3_location, Source2_location)
ans = isLeft(a_c3s2, b_c3s2, (300,100))
print(ans)

print("Cask3-Source3 line params")
a_c3s3, b_c3s3 = findLinePts(Cask3_location, Source3_location)
ans = isLeft(a_c3s3, b_c3s3, (300,100))
print(ans, '\n')



####################### Calculate source & cask distances to every point
Xpts = range(room_length)
Ypts = range(room_width)
xv,yv = np.meshgrid(Xpts, Ypts, sparse=False, indexing='xy')
coords = np.column_stack((xv.ravel(),yv.ravel()))
##coords = list(itertools.product(x_points, y_points))
#print(xv)

d_source1 = np.zeros([room_width, room_length])
d_source2 = np.zeros([room_width, room_length])
d_source3 = np.zeros([room_width, room_length])

d_cask1s1 = np.zeros([room_width, room_length])
d_cask1s2 = np.zeros([room_width, room_length])
d_cask1s3 = np.zeros([room_width, room_length])

d_cask2s1 = np.zeros([room_width, room_length])
d_cask2s2 = np.zeros([room_width, room_length])
d_cask2s3 = np.zeros([room_width, room_length])

d_cask3s1 = np.zeros([room_width, room_length])
d_cask3s2 = np.zeros([room_width, room_length])
d_cask3s3 = np.zeros([room_width, room_length])

for x in Xpts:
    for y in Ypts:
        point = np.array([xv[y,x], yv[y,x]])
        d_source1[y,x] = np.linalg.norm(point - Source1_location)
        d_source2[y,x] = np.linalg.norm(point - Source2_location)
        d_source3[y,x] = np.linalg.norm(point - Source3_location)
        
        # Casks attenuate away from each source 
        # Cask 1 
        if not isLeft(a_c1s1, b_c1s1, (x,y)):  # Source 1 
            d_cask1s1[y,x] = np.linalg.norm(point - Cask1_location)
        if     isLeft(a_c1s2, b_c1s2, (x,y)):  # Source 2 
            d_cask1s2[y,x] = np.linalg.norm(point - Cask1_location)
        if     isLeft(a_c1s3, b_c1s3, (x,y)):  # Source 3
            d_cask1s3[y,x] = np.linalg.norm(point - Cask1_location)
        
        # Cask 2
        if not isLeft(a_c2s1, b_c2s1, (x,y)):  # Source 1
            d_cask2s1[y,x] = np.linalg.norm(point - Cask2_location)
        if not isLeft(a_c2s2, b_c2s2, (x,y)):  # Source 2
            d_cask2s2[y,x] = np.linalg.norm(point - Cask2_location)
        if     isLeft(a_c2s3, b_c2s3, (x,y)):  # Source 3
            d_cask2s3[y,x] = np.linalg.norm(point - Cask2_location)
        
        # Cask 3
        if not isLeft(a_c3s1, b_c3s1, (x,y)):  # Source 1
            d_cask3s1[y,x] = np.linalg.norm(point - Cask3_location)
        if not isLeft(a_c3s2, b_c3s2, (x,y)):  # Source 2
            d_cask3s2[y,x] = np.linalg.norm(point - Cask3_location)
        if     isLeft(a_c3s3, b_c3s3, (x,y)):  # Source3
            d_cask3s3[y,x] = np.linalg.norm(point - Cask3_location)

##print(d_source1)

###### ADJUST CASK DISTANCES
# Set to 0 if over 1.5x20 (radius of cask)
# Points on source side of cask should already be 0
def threshold(d_cask):
    threshold_indices = d_cask > (1.5*Cask_rad*2)
    d_cask[threshold_indices] = 0
    return d_cask

d_cask1s1 = threshold(d_cask1s1)
d_cask1s2 = threshold(d_cask1s2)
d_cask1s3 = threshold(d_cask1s3)

d_cask2s1 = threshold(d_cask2s1)
d_cask2s2 = threshold(d_cask2s2)
d_cask2s3 = threshold(d_cask2s3)

d_cask3s1 = threshold(d_cask3s1)
d_cask3s2 = threshold(d_cask3s2)
d_cask3s3 = threshold(d_cask3s3)



####################### Densities ######################################
argon_density= 0.001784 # Dry air near sea level in g cm3
lead_density = 11.36
concrete_density = 3.15
#################### Interaction Coefficients FROM TABLES  #######################
cesium_137 = np.array([0.662,8.04e-2,7.065e-2,0.6,0.8]) #mass interaction coefficient[MeV,mass interactions ( cm2/g),from Faw and Shultis,energis from Faw and Shultis]
concrete = np.array([0.662,8.062e-2,6.083e-2, 0.6,0.8])
lead = np.array([0.662,1.167e-1,8.408e-2, 0.6,0.8])
####################### Function for extrapolating attenuation
####################### Coefficients from NIST tables
def extrapolation(isotope):
    y1,y3,x1,x2,x3=isotope[1],isotope[2],isotope[3],isotope[0],isotope[4]
    k=(y3-y1)/(x3-x1)
    y2=(k*(x2-x1))+y1
    return y2

###################### GammaEnergy = np.array([0.662]) # MeV, characteristic gamma for Cs 137
###################### For hot cell there will be multiple isotopes with different characteristic energies
#####################  0.662 Mev Interaction Coefficients
interaction_coefficient = np.array([extrapolation(cesium_137)]) #cm2/g
lead_interaction_coefficient = np.array([extrapolation(lead)]) #cm2/g
concrete_interaction_coefficient = np.array([extrapolation(concrete)]) #cm2/g
##################### Total Miu
total_miu = interaction_coefficient*argon_density # 1/cm
lead_total_miu = lead_interaction_coefficient*lead_density # 1/cm
concrete_total_miu = concrete_interaction_coefficient*concrete_density # 1/cm
##################### Source Strength
##
source1_strength= (9.44601235e+14/3)/(((4* np.pi)*(d_source1**2)))
source2_strength= (9.44601235e+14/6)/(((4* np.pi)*(d_source2**2)))
source3_strength= (9.44601235e+14/2)/(((4* np.pi)*(d_source3**2)))
##
Source_Strength = 9.44601235e+17/(((4* np.pi)*(R**2)))# particles/cm2 isotropic source emmiting 1e24 particles at origin
##
#################### Attenuations
source1attenuation_at_R= np.exp(-(total_miu*np.abs(d_source1)))  # ARGON ATTENUATION AT ALL POINTS IN THE ROOM FOR SOURCE 1
source2attenuation_at_R= np.exp(-(total_miu*np.abs(d_source2)))  # ARGON ATTENUATION AT ALL POINTS IN THE ROOM FOR SOURCE 2
source3attenuation_at_R= np.exp(-(total_miu*np.abs(d_source3)))  # ARGON ATTENUATION AT ALL POINTS IN THE ROOM FOR SOURCE 3

# LEAD ATTENUATION FROM CENTER OF CASK 1
cask1attenuation= np.exp(-(lead_total_miu*np.abs(d_cask1s1))) + np.exp(-(lead_total_miu*np.abs(d_cask1s2))) + np.exp(-(lead_total_miu*np.abs(d_cask1s3)))  
# LEAD ATTENUATION FROM CENTER OF CASK 2
cask2attenuation= np.exp(-(lead_total_miu*np.abs(d_cask2s1))) + np.exp(-(lead_total_miu*np.abs(d_cask2s2))) + np.exp(-(lead_total_miu*np.abs(d_cask2s3)))  
# LEAD ATTENUATION FROM CENTER OF CASK 3
cask3attenuation= np.exp(-(lead_total_miu*np.abs(d_cask3s1))) + np.exp(-(lead_total_miu*np.abs(d_cask3s2))) + np.exp(-(lead_total_miu*np.abs(d_cask3s3)))  

totalcaskattenuation=cask1attenuation+cask2attenuation+cask3attenuation
##
attenuation_at_R= np.exp(-(total_miu*np.abs(R)))
##
ResponseFunction=np.array(1.835e-8*cesium_137[0]*interaction_coefficient) #R/cm2, R=roentgen
Exposure_rate=ResponseFunction*Source_Strength*attenuation_at_R # R
##
Exposure_rate_source1=ResponseFunction*source1_strength*source1attenuation_at_R # R
Exposure_rate_source2=ResponseFunction*source2_strength*source2attenuation_at_R # R
Exposure_rate_source3=ResponseFunction*source3_strength*source3attenuation_at_R # R

Total_Exposure=(Exposure_rate_source1+Exposure_rate_source2+Exposure_rate_source3)*totalcaskattenuation
EX=np.ma.array(Total_Exposure)
##print ("\nTotal exposure")
##print (Total_Exposure)


#################### Exporting total exposure into file
exposure_dict = {}
# Discretize here as needed
grid = 1 #cm
Xgrid = range(0, room_length, grid)
Ygrid = range(0, room_width, grid)

for i in Xgrid:
    for j in Ygrid:
        exposure = Total_Exposure[j,i]
        if exposure==float("inf"):  # dict reader can't handle inf so replace with max exposure
            maxVal = Total_Exposure[np.isfinite(Total_Exposure)].max()
            #print(maxVal)
            exposure = maxVal
            #exposure = sys.float_info.max
        exposure_dict[(i,j)] = exposure

# print(exposure_dict)
f = open("Total_Exposure.txt", "w")
f.write(str(exposure_dict))
f.close()


################## Plotting exposure heatmaps
#print("Exposure_rate: ", Exposure_rate)
#print (Exposure_rate.shape)
r = np.ptp(Exposure_rate,axis=1)
#print("Exposure_rate ranges", r)

# fig1, ax1 = plt.subplots()
# #im = ax1.imshow(Exposure_rate)
# #im = ax1.matshow(Exposure_rate, cmap=cm.gray_r, norm=LogNorm(vmin=0.01, vmax=1))
# im = ax1.imshow(Exposure_rate, norm=LogNorm())
# ax1.set_title('Exposure rate in log scale (scenario 1)')
# ax1.autoscale_view()
# cb = fig1.colorbar(im)
# cb.set_label("Exposure rate (R/hr)")


fig2, ax2 = plt.subplots()
im = ax2.imshow(Total_Exposure, norm=LogNorm())
ax2.set_title('Total exposure rate in log scale (scenario 2)')
cb = fig2.colorbar(im)
cb.set_label("Exposure rate (R/hr)")
ax2.add_patch(robotpatch)
ax2.add_patch(TBpatch)
ax2.add_patch(source1patch)
ax2.add_patch(source2patch)
ax2.add_patch(source3patch)
ax2.add_patch(sourcedrycast1)
ax2.add_patch(sourcedrycast2)
ax2.add_patch(sourcedrycast3)
ax2.autoscale_view()
plt.xlim(0,room_length)
plt.ylim(0,room_width)
##plt.plot(ax2)
##plt.plot(ax)
########### PLOTTING THE SOURCE ################
#fig2, ax2 = plt.subplots()
#circle=Path.circle((75,75),radius=170,readonly=False)
#sourcepatch = PathPatch(circle, facecolor='None', edgecolor='green')
#ax2.add_patch(sourcepatch)
#ax2.set_title('Map Space')
#ax2.autoscale_view()
#plt.xlim(0,300)
#plt.ylim(0,200)
# fig3, ax3 = plt.subplots()
# ax3.plot(EX)
# plt.xlim(0,room_length)
# plt.ylim(0,room_width)
plt.show()

