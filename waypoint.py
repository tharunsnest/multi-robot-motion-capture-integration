#!/usr/bin/env python

import rospy
import numpy as np
from numpy import pi
#from scipy.spatial import distance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
#PI = 3.1415926535
theta_init = pi/2
class move2goal:
    def __init__(self):
        rospy.init_node('create2_bot',anonymous=True)
        self.v_pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    
        self.source_pose_pub = rospy.Publisher('s_pose',Pose2D,queue_size=10)
        self.source_pose_sub = rospy.Subscriber('s_pose',Pose2D,self.get_source) # needs to be changed to Pose from Pose2D

        self.dest_pose_pub = rospy.Publisher('d_pose',Pose2D,queue_size=10)
        self.dest_pose_sub = rospy.Subscriber('d_pose',Pose2D,self.get_dest) # needs to be changed to Pose from Pose2D
    
        self.s_pose = Pose2D()
        self.s_pose.x = 0
        self.s_pose.y = 0
        self.s_pose.theta = theta_init

        self.d_pose = Pose2D()
        self.d_pose.x = 0
        self.d_pose.y = 0
        self.d_pose.theta = theta_init

        self.rate = rospy.Rate(4)
    
    def get_source(self,data):
        self.s_pose.x = data.x
        self.s_pose.y = data.y
        self.s_pose.theta = data.theta
    
    def get_dest(self,data):
        self.d_pose.x = data.x
        self.d_pose.y = data.y
        self.d_pose.theta = data.theta
        self.waypoint((self.s_pose.x,self.s_pose.y),(self.d_pose.x,self.d_pose.y))


    def waypoint(self,p,p_f):
        p = np.array(p)
        p_f = np.array(p_f)#could be preprocessed and sent to waypoint
        v = self.velocity_vector(p,p_f) #should be in terms of r,theta
        self.rotate(v[1])
        self.move(v[0])
        
    #    p_t = get_position() #should be inside move()
    #    publish position to P_T
    #    p = p_t

    def move(self,distance):
        t = Twist()
        speed = 0.1
        #distance = 0.5
        t.linear.x = speed
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance < distance):
            #Publish the velocity
            self.v_pub.publish(t)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            self.rate.sleep()
            
        #After the loop, stops the robot
        t.linear.x = 0
        #Force the robot to stop
        self.v_pub.publish(t)
        # self.pose.x = distance*np.cos(self.pose.theta)
        # self.pose.y = distance*np.sin(self.pose.theta)
        # self.pose_pub.publish(self.pose)

    def rotate(self,relative_angle):
        t = Twist()
        #relative_angle = PI/2
        current_angle = 0
        if relative_angle<0 : #clockwise
            angular_speed = -0.05
        else:                 #anti_clockwise
            angular_speed = 0.05
        t.angular.z = angular_speed
        t0 = rospy.Time.now().to_sec()
        
        while(current_angle < abs(relative_angle)):
            self.v_pub.publish(t)
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed)*(t1-t0)
            self.rate.sleep()

        t.angular.z = 0
        self.v_pub.publish(t)
        self.s_pose.theta = self.s_pose.theta + current_angle * np.sign(relative_angle) 
        # self.pose_pub.publish(self.pose)

    


    def velocity_vector(self,p,p_f):
        #r = distance.euclidean(p, p_f)
        v = [0,0]
        v[0] = np.linalg.norm(p-p_f)
        p_vector = np.array([p_f[0]-p[0],p_f[1]-p[1]]) #direction vector
        theta = self.s_pose.theta
        o_vector = np.array([np.cos(theta),np.sin(theta)]) #orientation of the robot
        v[1] = np.arctan2(o_vector[0]*p_vector[1]-o_vector[1]*p_vector[0],o_vector[0]*p_vector[0]+o_vector[1]*p_vector[1]) #takes care of clockwise and anti clockwise
        #update_theta(v[1])
        return v


if __name__ == '__main__':
    try:
        x = move2goal()
        x.waypoint((0,0),(0.1,0))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
