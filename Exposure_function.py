from pylab import *
import matplotlib.pyplot as plt
import numpy as np

air_density= 1.205e-3 # Dry air near sea level in g cm3

#####################Function for extrapolating attenuation
#####################coefficients from NIST tables
def extrapolation(isotope):
    y1,y3,x1,x2,x3=isotope[1],isotope[2],isotope[3],isotope[0],isotope[4]
    k=(y3-y1)/(x3-x1)
    y2=(k*(x2-x1))+y1
    return y2


######################Defining the map #########################
room_length=3 # cm
room_width=4 # cm
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
total_miu= interaction_Coefficient*air_density # 1/cm
Source_Strength = 1e24/((4* np.pi)*(R**2))# particles/cm2 isotropic source emmiting 1e24 particles at origin
attinuation_at_R= np.exp(-total_miu*np.abs(R)) 
ResponseFunction=np.array(1.835e-8*cesium_137[0]*interaction_Coefficient) #R/cm2, R=roentgen
Exposure=ResponseFunction*Source_Strength*attinuation_at_R # R
print(Exposure)
print (points)

