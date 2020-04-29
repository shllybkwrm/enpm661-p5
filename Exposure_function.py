from pylab import *
from matplotlib.path import Path
from matplotlib.patches import PathPatch
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
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
circle=Path.circle(Source1_location, radius=1)
source1patch=PathPatch(circle, facecolor='red', edgecolor='green')
#################### PLOTTING SOURCE 2
circle=Path.circle(Source2_location, radius=1)
source2patch=PathPatch(circle, facecolor='red', edgecolor='green')
#################### PLOTTING SOURCE 3
circle=Path.circle(Source3_location, radius=1)
source3patch=PathPatch(circle, facecolor='red', edgecolor='green')
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
TB_x_coord=400  #LOCATION OF TOOL BOX center
TB_y_coord=40  #LOCATION OF TOOL BOX center
TB_length= 200
TB_height= 10
TB_points, TBvertices=make(TB_x_coord,TB_y_coord,TB_length,TB_height)
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


####################### Helper funcs for half-planes determined by cask-source lines
# https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
#  Uses cross-product/2D determinant
def isLeft(a, b, c):  # a,b two points on ends of line, c is new point to check
    return ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) > 0

### Find parameters of lines bisecting casks
# Source 1 - Cask 1
# perpendicular slope = -1/m
m_1 = -1/((Source1_location[1] - Cask1_location[1]) / (Source1_location[0] - Cask1_location[0]))
b_1 = Cask1_location[1] - m_1*Cask1_location[0]  # y-intercept b=y-mx
x0_1 = (0-b_1)/m_1  # x-intercept
xmax_1 = (room_width-b_1)/m_1
print("Source1-Cask1 line params")
print(m_1, b_1, x0_1, xmax_1)
a_1 = (x0_1,0)  # left
b_1 = (xmax_1,room_width)  # right
ans = isLeft(a_1, b_1, (300,100))
print(ans)

# Source 2 - Cask 2
m_2 = -1/((Source2_location[1] - Cask2_location[1]) / (Source2_location[0] - Cask2_location[0]))
b_2 = Cask2_location[1] - m_2*Cask2_location[0]  # y-intercept
x0_2 = (0-b_2)/m_2  # x-intercept
xmax_2 = (room_width-b_2)/m_2
print("Source2-Cask2 line params")
print(m_2, b_2, x0_2, xmax_2)
a_2 = (x0_2,0)  # right
b_2 = (xmax_2,room_width)  # left
ans = isLeft(a_2, b_2, (300,100))
print(ans)

# Source 3 - Cask 3
m_3 = -1/((Source3_location[1] - Cask3_location[1]) / (Source3_location[0] - Cask3_location[0]))
b_3 = Cask3_location[1] - m_3*Cask3_location[0]  # y-intercept
x0_3 = (0-b_3)/m_3  # x-intercept
xmax_3 = (room_width-b_3)/m_3
print("Source2-Cask2 line params")
print(m_3, b_3, x0_3, xmax_3)
a_3 = (x0_3,0)  # right
b_3 = (xmax_3,room_width)  # left
ans = isLeft(a_3, b_3, (300,100))
print(ans)



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
d_cask1 = np.zeros([room_width, room_length])
d_cask2 = np.zeros([room_width, room_length])
d_cask3 = np.zeros([room_width, room_length])
for x in Xpts:
    for y in Ypts:
        point = np.array([xv[y,x], yv[y,x]])
        d_source1[y,x] = np.linalg.norm(point - Source1_location)
        d_source2[y,x] = np.linalg.norm(point - Source2_location)
        d_source3[y,x] = np.linalg.norm(point - Source3_location)
        
        # Casks attenuate perpendicular to each source - still need to add more matrices 
        if not isLeft(a_1, b_1, (x,y)):
            d_cask1[y,x] = np.linalg.norm(point - Cask1_location)
        if not isLeft(a_2, b_2, (x,y)):
            d_cask2[y,x] = np.linalg.norm(point - Cask2_location)
        if isLeft(a_3, b_3, (x,y)):
            d_cask3[y,x] = np.linalg.norm(point - Cask3_location)

##print(d_source1)

###### ADJUST CASK DISTANCES
# Set to 0 if over 1.5x20 (radius of cask)
# Points on source side of cask should already be 0
threshold_indices = d_cask1 > (1.5*Cask_rad*2)
d_cask1[threshold_indices] = 0
threshold_indices = d_cask2 > (1.5*Cask_rad*2)
d_cask2[threshold_indices] = 0
threshold_indices = d_cask3 > (1.5*Cask_rad*2)
d_cask3[threshold_indices] = 0


###################### GammaEnergy = np.array([0.662]) # MeV, characteristic gamma for Cs 137
###################### For hot cell there will be multiple isotopes with different characteristic energies
#####################  0.662 Mev Interaction Coefficients
interaction_Coefficient= np.array([extrapolation(cesium_137)]) #cm2/g
concrete_interaction_Coefficient= np.array([extrapolation(concrete)]) #cm2/g
lead_interaction_Coefficient= np.array([extrapolation(lead)]) #cm2/g
##################### Total Miu
total_miu= interaction_Coefficient*argon_density # 1/cm
lead_total_miu= interaction_Coefficient*lead_density # 1/cm
concrete_total_miu= interaction_Coefficient*concrete_density # 1/cm
##################### Source Strength
##
source1_strength= (9.44601235e+14/3)/(((4* np.pi)*(d_source1**2)))
source2_strength= (9.44601235e+14/6)/(((4* np.pi)*(d_source2**2)))
source3_strength= (9.44601235e+14/2)/(((4* np.pi)*(d_source3**2)))
##
Source_Strength = 9.44601235e+17/(((4* np.pi)*(R**2)))# particles/cm2 isotropic source emmiting 1e24 particles at origin
##
#################### Attenuations
source1attenuation_at_R= np.exp(-(total_miu*np.abs(d_source1)))# ARGON ATTENUATION AT ALL POINTS IN THE ROOM FOR SOURCE 1
source2attenuation_at_R= np.exp(-(total_miu*np.abs(d_source2)))# ARGON ATTENUATION AT ALL POINTS IN THE ROOM FOR SOURCE 2
source3attenuation_at_R= np.exp(-(total_miu*np.abs(d_source3)))# ARGON ATTENUATION AT ALL POINTS IN THE ROOM FOR SOURCE 3
cask1attenuation= np.exp(-(lead_total_miu*np.abs(d_cask1)))# LEAD ATTENUATION FROM CENTER OF CASK 1
cask2attenuation= np.exp(-(lead_total_miu*np.abs(d_cask2)))# LEAD ATTENUATION FROM CENTER OF CASK 2
cask3attenuation= np.exp(-(lead_total_miu*np.abs(d_cask3)))# LEAD ATTENUATION FROM CENTER OF CASK 3
totalcaskattenuation=cask1attenuation+cask2attenuation+cask3attenuation
##
attenuation_at_R= np.exp(-(total_miu*np.abs(R)))
##
ResponseFunction=np.array(1.835e-8*cesium_137[0]*interaction_Coefficient) #R/cm2, R=roentgen
Exposure_rate=ResponseFunction*Source_Strength*attenuation_at_R # R
##
Exposure_rate_source1=ResponseFunction*source1_strength*source1attenuation_at_R # R
Exposure_rate_source2=ResponseFunction*source2_strength*source2attenuation_at_R # R
Exposure_rate_source3=ResponseFunction*source3_strength*source3attenuation_at_R # R
Total_Exposure=(Exposure_rate_source1+Exposure_rate_source2+Exposure_rate_source3)*totalcaskattenuation
EX=np.ma.array(Total_Exposure)
##print ("\nTotal exposure")
##print (Total_Exposure)
##
##
#################### Exporting total exposure into file
exposure_dict = {}
# Discretize every 5cm
Xgrid = range(0, room_length, 5)
Ygrid = range(0, room_width, 5)

for i in Xgrid:
    for j in Ygrid:
        exposure_dict[(i,j)] = Total_Exposure[j,i]

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
plt.xlim(0,732)
plt.ylim(0,152)
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
# plt.xlim(0,732)
# plt.ylim(0,152)
# plt.show()
