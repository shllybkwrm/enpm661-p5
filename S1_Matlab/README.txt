Chris Wheatley and Group 22
University of Maryland (College Park)
ENPM661 Spring 2020
Dr. Monfaredi
Project #5: Scenario 1

Scenario Description:
	Scenario 1 leverages RRT and non-holonomic differential drive to illustate how a RICA robot explores a wide outdoor map with many obstacles in order to locate the source of nuclear radiation.  

This scenario contains the following files found on GitHub at https://github.com/shllybkwrm/enpm661-p5/tree/master/S1_Matlab:
	- Temporary Places.kmz
		o This is the original Google Earth file where the map and obstacles were initially sketched out
	- calc_polygon_center.m
	- changeAngle.m
	- enu2lla.m
	- extrapolation.m
	- findLinePts.m
	- isLeft.m
	- obstacleCheckRigid.m
	- project5_scenario1_main.m
		o This is the main script and the only script that the user must run 
	- vrep_pioneer_child_script_scenario1_source_top_left.txt
		o This is the threaded child script for the Pioneer robot for the example where the goal is in the top LEFT of the map.
		o The code for this is already encoded in the "vrep_scenario_1_scene_source_top_left.ttt" scene, so if you run the simulation directly from the VREP scene, this code will be executed.
	- vrep_pioneer_child_script_scenario1_source_top_right.txt
		o This is the threaded child script for the Pioneer robot for the example where the goal is in the top RIGHT of the map.
		o The code for this is already encoded in the "vrep_scenario_1_scene_source_top_right.ttt" scene, so if you run the simulation directly from the VREP scene, this code will be executed.
	- vrep_scenario_1_scene_source_top_left.ttt
		o This is the VREP scene for the example where the goal is located in the top LEFT corner of the map
	- vrep_scenario_1_scene_source_top_right.ttt
		o This is the VREP scene for the example where the goal is located in the top RIGHT corner of the map
	- README.txt

Please visit the links below to view the MATLAB and VREP simulations:
	MATLAB Simulation Videos: https://umd.box.com/s/k96s2mcszuuqy41rxssqfavtir096cqo
	VREP Simulation Videos: https://umd.box.com/s/qlzooovropvz5scutbxqq7a23kbv7skz
			
NOTES:
	- The .ttt files, .txt files, and .mp4 files ending with "...source_top_right" all correspond to the exact same example where the goal is loacted in the top RIGHT corner of the map.
	- The .ttt files, .txt files, and .mp4 files ending with "...source_top_left" all correspond to the exact same example where the goal is loacted in the top LEFT corner of the map.
	- The MATLAB simulation assumes the RICA robot is being used, but we couldn't find a 3D model for this robot, so we ended up using the Pioneer robot for the VREP simulations (due to similar dimensions).
	- The paths in the simulation videos are by no means optimal (i.e. conequence of using RRT).  These are just examples.


If you want to run a simulation in MATLAB, and then in VREP, please follow the steps below:
	1) Ensure that MATLAB 2020a is installed on your machine (necessary for the lat/lon mapping executed at the end of the main script).
	2) Download all Project 5 Scenario 1 contents into a newly created local directory.
	3) In MATLAB, change the working directory to the working directory created in step #2.
	4) Run "project5_scenario1_main.m"
	5) Prompts will appear soliciting the following information:
		- Start Node Location consisting of x and y (meters) and orientation (degrees) (e.g. "[12, 15, 30]")
		- Selected Example (e.g. type "1" to evaluate the example where the goal is in the top left, and enter a "2" to evaluate the example where the goal is in the top right).  
			o Note that goal node cannot be specified by the user since the map was very carefully precomputed where radiation was assigned to each point on the map based on distance away from the goal (i.e. radiation source).
		- Obstacle Clearance (in meters)
		- RICA Robot Wheel RPMs (e.g. "[8,8]")
			o It is advised to NOT use RPMs greater than this, otherwise the path will likely be comprised of recursive loops, which is infeasible for realistic path planning.
	6) If there are conflicts with any of the user-specified inputs, you'll see warnings in the Command Window and will be prompted to re-enter valid inputs.
	7) A figure will then appear (feel free to manually maximize the .fig window size) showing RRT in action to ecplorre the map.  Radnom samples (big red dots) rapidly appear and disappear, with dotted red lines connecting the sample to the "least-cost" node on the graph.
	8) When the exploration is over, a green line will be plotted showing the "optimal" path.
	9) A separte figure will appear showing the determined path plotted with blue markers on latitude/longitude axes atop a topograohical map showing buildings and parking lots, etc.
	10 Lastly, note that there is information output into the MATLAB Command Window. This information provides the direct inputs needed to run the corresponding VREP simulation.  
	Copy all of this information as it is printed out, except for Elapsed Time, and feel free to paste into lines 3-8 of the threaded robot child script in the corresponding VREP scene attached.
		- For Example, if you ran the MATLAB example where the goal is in the top RIGHT: 
		1) Copy the printed lines in the MATLAB Command Window for x_vals,y_vals,left_vels,right_vels,orientations,exposures  
		2) Open the "vrep_scenario_1_scene_source_top_right.ttt", open the threaded child script for the Pioneer robot in the left-hand heirarchy viewer, and paste in the copied tect into lines 3-8.
		3) Save this modified script (Ctrl+S), exit out and go back to the orginal VREP scene, and click the green play button to start the simulation.
		4) The simulation will not end on its own, so please end it manually once the robot reaches the goal. 
			-- NOTE: The small green cylinders in the VREP scenes represent the goal locations.  The locations of these cylinders are not 100% accurate, just visual indicators for the user. 
	
