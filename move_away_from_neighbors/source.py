#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion
#from threading import Thread
#import time

n = 10 #number of robots need to edit the main function whenever changed

Arr_s =[(1,1)]*n



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

def main():
	global Arr_s
	
	#rospy.init_node('source_pub',disable_signals=True)
	#rate = rospy.Rate(100)
	robot1_pose_pub = rospy.Publisher('robot1/s_pose',Pose2D,queue_size=10)
	robot2_pose_pub = rospy.Publisher('robot2/s_pose',Pose2D,queue_size=10)
	robot3_pose_pub = rospy.Publisher('robot3/s_pose',Pose2D,queue_size=10)
	robot4_pose_pub = rospy.Publisher('robot4/s_pose',Pose2D,queue_size=10)
	robot5_pose_pub = rospy.Publisher('robot5/s_pose',Pose2D,queue_size=10)
	robot6_pose_pub = rospy.Publisher('robot6/s_pose',Pose2D,queue_size=10)
	robot7_pose_pub = rospy.Publisher('robot7/s_pose',Pose2D,queue_size=10)
	robot8_pose_pub = rospy.Publisher('robot8/s_pose',Pose2D,queue_size=10)
	robot9_pose_pub = rospy.Publisher('robot9/s_pose',Pose2D,queue_size=10)
	robot10_pose_pub = rospy.Publisher('robot10/s_pose',Pose2D,queue_size=10)
	
	robot1 = Pose2D()
	robot2 = Pose2D()
	robot3 = Pose2D()
	robot4 = Pose2D()
	robot5 = Pose2D()	
	robot6 = Pose2D()
	robot7 = Pose2D()
	robot8 = Pose2D()
	robot9 = Pose2D()
	robot510 = Pose2D()
	

	while not rospy.is_shutdown():
		robot1 = source('Robot1')
		robot2 = source('Robot2')
		robot3 = source('Robot3')
		robot4 = source('Robot4')
		robot5 = source('Robot5')
		robot6 = source('Robot6')
		robot7 = source('Robot7')
		robot8 = source('Robot8')
		robot9 = source('Robot9')
		robot10 = source('Robot10')
		
		robot1_pose_pub.publish(robot1)
		robot2_pose_pub.publish(robot2)
		robot3_pose_pub.publish(robot3)
		robot4_pose_pub.publish(robot4)
		robot5_pose_pub.publish(robot5)
		robot6_pose_pub.publish(robot6)
		robot7_pose_pub.publish(robot7)
		robot8_pose_pub.publish(robot8)
		robot9_pose_pub.publish(robot9)
		robot10_pose_pub.publish(robot10)

		Arr_s[0] = (robot1.x,robot1.y)
		Arr_s[1] = (robot2.x,robot2.y)
		Arr_s[2] = (robot3.x,robot3.y)
		Arr_s[3] = (robot4.x,robot4.y)
		Arr_s[4] = (robot5.x,robot5.y)
		Arr_s[5] = (robot6.x,robot6.y)
		Arr_s[6] = (robot7.x,robot7.y)
		Arr_s[7] = (robot8.x,robot8.y)
		Arr_s[8] = (robot9.x,robot9.y)
		Arr_s[9] = (robot10.x,robot10.y)
		
		
		
		
#		Arr_s.append()
#		Arr_s.append((robot1.x,robot1.y))
		
		# for i in range(n):
			
		# 	Arr_s.append((robo))

"""
def prin():
	
	for i in range(10):
		print(Arr_s)
		time.sleep(1)
	

if __name__ == '__main__':
	global Arr_s
	
	#t1 = Thread(target = prin)
	
	#t1.start()
	main()
	#t1.join()
	
"""	
		
		
	
	
	



