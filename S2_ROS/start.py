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
