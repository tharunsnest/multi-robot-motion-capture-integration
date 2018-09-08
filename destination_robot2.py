#!/usr/bin/env python

import numpy as np
import waypoint
import rospy
from geometry_msgs.msg import Pose2D


d_sample = 1.0 #takes care of drift
d_threshold= 0.1 #tolerance with the destination

dest= [(5.0,0.0)] #destination coordinates


def get_source(data):
    s_pose.x = data.x
    s_pose.y = data.y
    s_pose.theta = data.theta 

s_pose = Pose2D()
s_pose.x = 0
s_pose.y = 0
s_pose.theta = 0
d_pose = Pose2D()
d_pose.x = 0
d_pose.y = 0
d_pose.theta = 0
#check for theta value

rospy.init_node('robot2_destinations')
rate = rospy.Rate(10)
dest_pose_p = rospy.Publisher('robot2/d_pose',Pose2D,queue_size=10)
source_pose_sub = rospy.Subscriber('robot2/s_pose',Pose2D,get_source)



for i in range(len(dest)):

	p_f = np.array(dest[i])

	p = np.array((s_pose.x,s_pose.y)) #get source

	d_pose.x = p_f[0]
	d_pose.y = p_f[1]
	dist = np.linalg.norm(p_f-p)

	if (dist > d_sample):
		cos_theta = np.dot(p_f-p,[1,0])/dist
		sin_theta = np.cross([1,0],p_f-p)/dist #will change for 3 dimentional vector
		r = dist/d_sample

		for j in range(int(r)):
		    d_pose.x = p[0] + (j+1)*d_sample*cos_theta
		    d_pose.y = p[1] + (j+1)*d_sample*sin_theta

		    dist_h = d_sample
		    p_fh = np.array((d_pose.x,d_pose.y))

		    while not rospy.is_shutdown():
				connections = dest_pose_p.get_num_connections()
				rospy.loginfo('connections: %d', connections)
				if connections > 0:
					dest_pose_p.publish(d_pose)
					break
				rate.sleep()

		    while (abs(dist_h) > d_threshold):
				p_h = np.array((s_pose.x,s_pose.y))
				dist_h = np.linalg.norm(p_h-p_fh)
		

		if (r-int(r)) > d_threshold :

			d_pose.x = d_pose.x + (r-int(r))*d_sample*cos_theta
			d_pose.y = d_pose.y + (r-int(r))*d_sample*sin_theta
			dist_h = (r-int(r))*d_sample

			while not rospy.is_shutdown():
				connections = dest_pose_p.get_num_connections()
				rospy.loginfo('connections: %d', connections)
				if connections > 0:
					dest_pose_p.publish(d_pose)
					break
				rate.sleep()

		#dest_pose_p.publish(d_pose)
		while (abs(dist_h) > d_threshold):
			p_h = np.array((s_pose.x,s_pose.y))
			dist_h = np.linalg.norm(p_h-p_fh)


	else :
		while not rospy.is_shutdown():
			connections = dest_pose_p.get_num_connections()
			rospy.loginfo('connections: %d', connections)
			if connections > 0:
				dest_pose_p.publish(d_pose)
				break
			rate.sleep()

		#dist=np.linalg.norm(p-p_f) 
		while (abs(dist) > d_threshold) :
			    #dest_pose_p.publish(d_pose)
			    #print 1
			    p = np.array((s_pose.x,s_pose.y))
			    print dist
			    dist = np.linalg.norm(p-p_f)


	

	    
