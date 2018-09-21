#import Array of source coordinates
import rospy
from destination import Arr_S
import numpy as np


def funct(a):
    alpha_sample = 0.1
    v = [0,0]
    for i in range(len(Arr_S)):
        v = v + a - Arr_S[i]
        v_h = v/np.linalg.norm(v)
        v_h = v_h * alpha_sample
    return a+v_h

def publishing(a):
    from geometry_msgs.msg import Pose2D
    d_pose = Pose2D()
    d_pose.x = a[0]
    d_pose.y = a[1]
    d_pose.theta = 0
    temp_str = 'dest_pub_'+str(a[2])
    rospy.init_node(temp_str)
    temp_str = 'robot'+str(a[2])+'/s_pose'
    dest_pose_p = rospy.Publisher(temp_str,Pose2D,queue_size=10)
    
    
    while not rospy.is_shutdown():
        connections = dest_pose_p.get_num_connections()
        #rospy.loginfo('connections: %d', connections)
        if connections > 0:
            dest_pose_p.publish(d_pose)
            break

    