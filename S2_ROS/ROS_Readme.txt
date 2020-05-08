#########START NEW TERMINAL######################
#####################CREATE CATKIN WORK SPACE
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/project5_ws/src
$ catkin_create_pkg project5_controller rospy
$ cd ~/project5_ws/
$ catkin_make
$ . ~/project5_ws/devel/setup.bash
$ echo $ROS_PACKAGE_PATH

##OUTPUT
/home/youruser/project5_ws/src:/opt/ros/kinetic/share
############Customizing Your Package
First update the description tag:
change name email and package heading
Next comes the maintainer tag
Next is the license tag, which is also required
Make sure all listed dependencies have been added as a build_depend and that catkin default build tool_depend. Also ensue all dependencies are exec_depend taged.
#############BUild your Package
source your environment setup file
$ source /opt/ros/kinetic/setup.bash 
$ cd ~/project5_ws/
$ ls src
-- You should see the package and CMakeLists.txt in here
$ catkin_make
###############SUBSCRIBE TO ODOM TO GET ROBOT LOCATION
###############COPY POINT ON FROM PATH WITH RESPECTIVE EXPOSURES
########PUBLISH POINTS, AND VELOCITYIS TO 'cmd_vel_mux/input/navi'
#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist  

def distance(x1,y1,x2,y2):
    xd= x1-x2
    yd= y1-y2
    return math.sqrt(xd*xd + yd*yd)

class ExposureCounter(object):
    def __init__(self, pub, locations): 
        self.pub = pub
        self.locations = locations

    def callback(self, msg): #what are we trying to extract from the message class
        global x
        global y
        global theta

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])    
        speed = Twist()
        r = rospy.Rate(4)
        exposure= 0
        for exp,l_x,l_y in self.locations:
            next_point = Point()
            next_point.x = l_x
            next_point.y = l_y
            inc_x = next_point.x - x
	    inc_y = next_point.y - y
	    angle_to_point = math.atan2 (inc_y, inc_x)     
            dist = distance(x, y, l_x, l_y)
            exposure=exp+exposure
            if abs(angle_to_point-theta) > 0:
               speed.linear.x = 0.0
               speed.angular.z = 0.3
            else:
               speed.linear.x = 0.5
               speed.angular.z = 0.0  
        self.pub.publish(speed)
        r.sleep()
        rospy.loginfo('dist: {}, angle: {}, theta: {}, exposure: {}'.format(dist,angle_to_point,theta,exposure))
#     rospy.loginfo('x: {}, y: {}, theta: {}, exposure: {}'.format(x,y,theta,exposure))
    

def main():
    rospy.init_node('location_monitor')
    locations = []
    locations.append((30,2.0,-0.24))
    locations.append((40,2.0,-0.24))
#    locations.append((, , ))
    
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    monitor = ExposureCounter(pub,locations)
    
    rospy.Subscriber("/odom", Odometry, monitor.callback)
    rospy.spin()



if  __name__ == '__main__':
     main()

########### SAVE THE CUSTOM WORLD IN A LOCATION AND GET IT'S PATH
CUSTOM WORLDS ARE singrm.world and singrm2.world
###########START NEW TERMINAL
###########CHANGE DIRECTORY TO WORK SPACE
$ cd ~/project5_ws/
######NOTE "project5 is the name of our workspace
######SOURCE YOUR BASH
$ source devel/setup.bash
###########LAUNCH ROS GAZEBO TURTLE BOT WITH WORLDFILE = CUSTOM WORLD
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/lu/project5_ws/src/prj5_controller/singrm2.world
#######CHANGE DIRECTORY TO LOCATION OF YOUR SCRIPT INSIDE THE PACKAGE
##########RUN THE SCRIPT
rosrun location_monitor start2.py

