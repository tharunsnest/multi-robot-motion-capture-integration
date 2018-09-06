#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D

d_sample = 1.0 #takes care of drift
d_threshold= 0.1 #tolerance with the destination

dest= [(1,0),(1,1),(0,1),(0,0),(1,0)] #destination coordinates


def get_source(data):
    s_pose.x = data.x
    s_pose.y = data.y
    s_pose.theta = data.theta 
#x= waypoint.move2goal()

rospy.init_node('robot1_destinations',anonymous=True)
dest_pose_pub = rospy.Publisher('robot1/d_pose',Pose2D,queue_size=10)
source_pose_sub = rospy.Subscriber('robot1/s_pose',Pose2D,get_source)
s_pose = Pose2D()
d_pose = Pose2D()
#check for theta value

	
for i in range(len(dest)):
    
    p_f = np.array(dest[i])
	
    p = np.array(s_pose.x,s_pose.y) #get source

    d_pose.x = p_f[0]
    d_pose.y = p_f[1]
    dist = np.linalg.norm(p-p_f)
    
    if (dist > d_sample):
        cos_theta = np.dot(p-p_f)/dist
        sin_theta = np.cross([0,1],p-p_f)/dist #will change for 3 dimentional vector
        r = dist/d_sample

        for j in range(int(r)):
            d_pose.x = p[0] + (j+1)*d_sample*cos_theta
            d_pose.y = p[1] + (j+1)*d_sample*sin_theta
            
            dest_pose_pub.publish(d_pose)

            dist_h = d_sample
            p_fh = np.array(d_pose.x,d_pose.y)
            
            while (abs(dist_h) > d_threshold):
                p_h = np.array(s_pose.x,s_pose.y)
                dist_h = np.linalg.norm(p_h-p_fh)
        
        d_pose.x = p[0] + (j+1)*(r-int(r))*d_sample*cos_theta
        d_pose.y = p[1] + (j+1)*(r-int(r))*d_sample*sin_theta
        dist_h = (r-int(r))*d_sample
        dest_pose_pub.publish(d_pose)
        while (abs(dist_h) > d_threshold):
                p_h = np.array(s_pose.x,s_pose.y)
                dist_h = np.linalg.norm(p_h-p_fh)


    else :
        dest_pose_pub.publish(d_pose)
        #dist=np.linalg.norm(p-p_f) 
        while (abs(dist) > d_threshold) :
            p = np.array(s_pose.x,s_pose.y)
            dist = np.linalg.norm(p-p_f)
    
