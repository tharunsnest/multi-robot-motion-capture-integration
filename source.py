#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
n = 2 #number of robots
Arr_s =[]

def source(model):
	#rospy.init_node('source_pub')
	robot = Pose2D()
	#robot2 = Pose2D()

	rospy.wait_for_service('/gazebo/get_model_state')
	model_service= rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

	r1 = model_service(model,'world')
	pose = r1.pose.position
	quad = r1.pose.orientation
	
	quad_list = [quad.x,quad.y,quad.z,quad.w]
	(roll,pitch,yaw) = euler_from_quaternion(quad_list)
	robot.x = pose.x
	robot.y = pose.y
	#robot.z = pose.z
	robot.theta = yaw
	
	return robot

if __name__ == '__main__':
	rospy.init_node('source_pub')
	rate = rospy.Rate(100)
	robot1_pose_pub = rospy.Publisher('robot1/s_pose',Pose2D,queue_size=10)
	robot2_pose_pub = rospy.Publisher('robot2/s_pose',Pose2D,queue_size=10)
	robot1 = Pose2D()
	robot2 = Pose2D()
	Arr_s = []

	while not rospy.is_shutdown():
		robot1 = source('Robot1')
		robot2 = source('Robot2')
		robot1_pose_pub.publish(robot1)
		robot2_pose_pub.publish(robot2)
		Arr_s.append((robot1.x,robot1.y))
		Arr_s.append((robot1.x,robot1.y))
		# for i in range(n):
			
		# 	Arr_s.append((robo))
		
		
	
	
	



