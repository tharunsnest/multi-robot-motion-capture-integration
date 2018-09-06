#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D

d_sample = 1.0
d_threshold= 0.1

dest= [(1,1)]

#x= waypoint.move2goal()

rospy.init_node('robot1_destinations',anonymous=True)
dest_pose_pub = rospy.Publisher('robot1/d_pose',Pose2D,queue_size=10)
source_pose_sub = rospy.Subscriber('robot1/s_pose',Pose2D,get_source)
s_pose = Pose2D()

def get_source(data):
    s_pose.x = data.x
    s_pose.y = data.y
    s_pose.theta = data.theta
	
for i in range(len(dest)):
    
    p_f = np.array(dest[i])
	
    p = np.array(s_pose.x,s_pose.y) #get source

    d_pose.x = p_f[0]
    d_pose.y = p_f[1]
    dist = np.linalg.norm(p-p_f)
    if (dist > d_sample):
	for j in range(dist/d_sample)

    dest_pose_pub.publish(d_pose)
    
    #dist=np.linalg.norm(p-p_f) 

    while (abs(dist) > d_threshold) :
        pass
  
    i = i+1
