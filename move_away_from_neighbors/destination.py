#!/usr/bin/env python


#from source import Arr_s
import numpy as np
#Arr_S =np.array(Arr_s)
#from multiprocessing import Pool
# from moveaway_h import funct #due to OS constraints, implemented in a separate script
# from moveaway_h import publishing

def _init_s(Arr_s):
    
    global Arr_S
    Arr_S = Arr_sh


def funct(a):
    alpha_sample = 1
    v = [0,0]
    for i in range(len(Arr_S)):
        v = v + a - Arr_S[i]
        v_h = v/np.linalg.norm(v)
        v_h = v_h * alpha_sample
    return a+v_h

# def check(a):
#     a = Arr_S(a[2]-1)

def publishing(a):
    if a[3]==0:
        return
    from geometry_msgs.msg import Pose2D
    import rospy
    d_pose = Pose2D()
    d_pose.x = a[0]
    d_pose.y = a[1]
    d_pose.theta = 0
    temp_str = 'dest_pub_'+str(a[2])
    rospy.init_node(temp_str)
    temp_str = 'robot'+str(a[2])+'/d_pose'
    dest_pose_p = rospy.Publisher(temp_str,Pose2D,queue_size=10)
#    check(a)
    
    while not rospy.is_shutdown():
        connections = dest_pose_p.get_num_connections()
        #rospy.loginfo('connections: %d', connections)
        if connections > 0:
            dest_pose_p.publish(d_pose)
            break

    global d
    d = [0,0]
    def get_source(data):
        global d
        d = [data.x,data.y]

    temp_str = 'robot'+str(a[2])+'/s_pose'

    source_pose_s = rospy.Subscriber(temp_str,Pose2D,get_source)

    while not rospy.is_shutdown():
        if (np.linalg.norm(d-[a[0],a[1]]) <= 0.1) :
            break
        
    


    

def boundary_conditions(x1,x2,y1,y2):
    global X1,X2,Y1,Y2
    X1 = x1
    X2 = x2
    Y1 = y1
    Y2 = y2

def check_for_boundary(d):
    if(X1<=d[0]<=X2):
        if(Y1<=d[1]<=Y2):
            return 1
    return 0
   
    

def main():

    import source
    print source.Arr_s
    Arr_sh = np.array(source.Arr_s)
    print(Arr_sh)
"""
    pool1 = Pool(initializer = _init_s, initargs= (Arr_sh,))
    Arr_D = pool1.map(funct,Arr_sh)
    pool1.close()
    pool1.join()
    
    pool2 = Pool(initializer=boundary_conditions,initargs=(-5,5,-5,5))
    check = pool2.map(check_for_boundary,Arr_D)
    pool2.close()
    pool2.join()
        
    pool3 = Pool()
    Arr_D_h = np.column_stack((Arr_D,np.arange(1,len(Arr_D)+1),check))
    
    pool3.map(publishing,Arr_D_h)
    pool3.close()
    pool3.join()
"""
if __name__ == '__main__':

    main()
