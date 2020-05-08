
RRT star program with non-holonomic constraints:
================================================

Description: 
============
This program uses RRT star algorithm to provide path planning solution to a robot exploration in a hot Cell Room. The main parameters used in this program are:
NUMBER_OF_NODES :This is parameter that can be adjusted to increase the number of samples to be used by RRT star. The more number of samples used will allow for a more optimal path. However, the optimality will be enhanced at the expense of more computation timedue to the increased number of samples. This paramter is located in Line 27 of the RRT_star_non_holonomic.py file. 
NN_Radius: It is an adjustable parameter that will allow for more nodes in the exploring tree to be considered when looking for minimum cost node among nearest nodes to new node. This parameter is located in Line 26 of the RRT_star_non_holonomic.py file. 

How to run Program:
===================
The program makes use of the following files:
1.RRT_star_non_holonomic.py
2.ReadExposureDict.py
3.prj5_map.py
4.TotalExposure.txt

To run this program you will need the following software:
Ubuntu 18.04
Python v3.6

You will also need the following python libraries:
math
random
numpy
matplotlib
enum

In order to run program make sure above file are located inside same folder. Then open a terminal window pressing CTRL + T and enter the command below:

$ python RRT_star_non_holonomic.py <enter>

This should start the program and the matplot graph will display how the tree is expanded along the map. You can also observe how rewiring is done in the map since the lines correspoding to rewiring are colored green.
At the end of the program completion a text file will be output that will contain the rrt star path nodes that will be used as input to the gazebo simulation. Also a total radiation exposure value will be output in this file to show how much radiation the robot was exposed to during its exploration.

EXECUTION TIME :
The algorithm takes about 1.5 hours when run for 400 random samples
The algorithm takes about 3 hours to complete running when used with 700 random samples.
Most of the time comsumption is due to the dubin path computation for each segment that forms the curved path between 2 nodes. Also the addition of rewiring increasing the amount of dubins path computation slowing down the computation time performanceof the program.

Test Cases:
The program is run for three different goal test points: (110,110), (320,125), (650,75).
Pictures and video of the algorithm performance for each test case, are provided along with the rest of the files.
