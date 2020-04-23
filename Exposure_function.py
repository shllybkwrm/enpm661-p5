from pylab import *
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import matplotlib.pyplot as plt
import numpy as np
import itertools

"""
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

def make(xcoord,ycoord,length, width):
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
Robot_points,rvertices=make(robot_x_coord,robot_y_coord,robot_breadth,robot_height)
########### PLOTTING THE ROBOT ################
rcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
robot = Path(rvertices,rcodes)
robotpatch = PathPatch(robot, facecolor='green', edgecolor='green')
#################### PLOTTING SOURCE 1
circle=Path.circle((366,76),radius=1,readonly=False)
source1patch=PathPatch(circle, facecolor='red', edgecolor='green')
#################### PLOTTING SOURCE 2
circle=Path.circle((182,76),radius=1,readonly=False)
source2patch=PathPatch(circle, facecolor='red', edgecolor='green')
#################### PLOTTING SOURCE 3
circle=Path.circle((548,76),radius=1,readonly=False)
source3patch=PathPatch(circle, facecolor='red', edgecolor='green')
#################### PLOTTING DRY STORAGE CASK
circle=Path.circle((200,100),radius=20,readonly=False)
sourcedrycast1=PathPatch(circle, facecolor='red', edgecolor='red')
#################### PLOTTING DRY STORAGE CASK
circle=Path.circle((400,100),radius=20,readonly=False)
sourcedrycast2=PathPatch(circle, facecolor='red', edgecolor='green')
#################### PLOTTING DRY STORAGE CASK
circle=Path.circle((600,100),radius=20,readonly=False)
sourcedrycast3=PathPatch(circle, facecolor='red', edgecolor='green')
#################### TOOL BOX COORDINATE
TB_x_coord=400  #LOCATION OF TOOL BOX center
TB_y_coord=40  #LOCATION OF TOOL BOX center
TB_length= 200
TB_height= 10
TB_points, TBvertices=make(TB_x_coord,TB_y_coord,TB_length,TB_height)
TB = Path(TBvertices,rcodes)
TBpatch = PathPatch(TB, facecolor='green', edgecolor='green')
######PLOTTING#####################
fig, ax = plt.subplots()
ax.add_patch(robotpatch)
ax.add_patch(TBpatch)
ax.add_patch(source1patch)
ax.add_patch(source2patch)
ax.add_patch(source3patch)
ax.add_patch(sourcedrycast1)
ax.add_patch(sourcedrycast2)
ax.add_patch(sourcedrycast3)
ax.set_title("Westinghouse Hot Cell")

ax.autoscale_view()
plt.xlim(0,732)
plt.ylim(0,152)
##plt.show()
argon_density= 0.001784 # Dry air near sea level in g cm3

##################### Functions for calculating source distances to every point
#def distance_2(p1,p2):
#    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
#    return distance
Xpts = range(732)
Ypts = range(152)
xv,yv = np.meshgrid(Xpts, Ypts, sparse=False, indexing='xy')
#coords = np.column_stack((xv.ravel(),yv.ravel()))
#coords = list(itertools.product(x_points, y_points))
#print(xv)
d_source1 = np.zeros([732,152])
d_source2 = np.zeros([732,152])
d_source3 = np.zeros([732,152])
for x in Xpts:
    for y in Ypts:
        point = np.array([xv[y,x], yv[y,x]])
        d_source1[x,y] = np.linalg.norm(point-[366,76])
        d_source2[x,y] = np.linalg.norm(point-[182,76])
        d_source3[x,y] = np.linalg.norm(point-[548,76])
##print(d_source1)
##print(d_source2)
##print(d_source3)

##################### Function for extrapolating attenuation
##################### Coefficients from NIST tables
def extrapolation(isotope):
    y1,y3,x1,x2,x3=isotope[1],isotope[2],isotope[3],isotope[0],isotope[4]
    k=(y3-y1)/(x3-x1)
    y2=(k*(x2-x1))+y1
    return y2


###################### Defining the map #########################
room_length= 732 # cm
room_width= 152 # cm
Source_location=[366,76]
x=np.linspace(-room_length/2,room_length/2,room_length+1)
y=np.linspace(-room_width/2,room_width/2,room_width+1)
X,Y = np.meshgrid(x,y, indexing='xy')
a=np.array(((0,X),(0,Y)))
R=np.linalg.norm(a) #The 2 norm of each point gives the distance from the center of the room
points = np.array(list(zip(X.flatten(),Y.flatten())))

#################### GammaEnergy = np.array([0.662]) # MeV, characteristic gamma for Cs 137
#################### For hot cell there will be multiple isotopes with different characteristic energies
cesium_137 = np.array([0.662,8.04e-2,7.065e-2,0.6,0.8]) #mass interaction coefficient[MeV,mass interactions ( cm2/g),from Faw and Shultis,energis from Faw and Shultis]
interaction_Coefficient= np.array([extrapolation(cesium_137)]) #cm2/g
total_miu= interaction_Coefficient*argon_density # 1/cm

source1_strength= 9.44601235e+17/(((4* np.pi)*(d_source1**2)))
source2_strength= 9.44601235e+17/(((4* np.pi)*(d_source2**2)))
source3_strength= 9.44601235e+17/(((4* np.pi)*(d_source3**2)))
Source_Strength = 9.44601235e+17/(((4* np.pi)*(R*R)))# particles/cm2 isotropic source emmiting 1e24 particles at origin


source1attenuation_at_R= np.exp(-(total_miu*np.abs(d_source1)))
source2attenuation_at_R= np.exp(-(total_miu*np.abs(d_source2)))
source3attenuation_at_R= np.exp(-(total_miu*np.abs(d_source3)))
attenuation_at_R= np.exp(-(total_miu*np.abs(R)))

ResponseFunction=np.array(1.835e-8*cesium_137[0]*interaction_Coefficient) #R/cm2, R=roentgen
Exposure_rate=ResponseFunction*Source_Strength*attenuation_at_R # R
Exposure_rate_source1=ResponseFunction*source1_strength*source1attenuation_at_R # R
Exposure_rate_source2=ResponseFunction*source2_strength*source2attenuation_at_R # R
Exposure_rate_source3=ResponseFunction*source3_strength*source3attenuation_at_R # R
Total_Exposure=Exposure_rate_source1+Exposure_rate_source2+Exposure_rate_source3
##print ("Exposure_rate_source1")
##print (Exposure_rate_source1)

################## Westinghhouse High-level Hot Cell
##print(X,"X")
##print(Y,"Y")
##print(R,"R")
print(Exposure_rate, "Exposure_rate")
##print(points)

########### PLOTTING THE SOURCET################
circle=Path.circle((75,75),radius=170,readonly=False)
sourcepatch = PathPatch(circle, facecolor='None', edgecolor='green')
fig1, ax = plt.subplots()
im = ax.imshow(Exposure_rate)
fig2, ax1 = plt.subplots()
ax1.add_patch(sourcepatch)
ax1.set_title('Map Space')

ax1.autoscale_view()
plt.xlim(0,300)
plt.ylim(0,200)
plt.show()
