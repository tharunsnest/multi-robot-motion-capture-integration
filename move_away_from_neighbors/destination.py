#!/usr/bin/env python


#from source import Arr_s

from threading import Thread
import rospy
import numpy as np
import time
#Arr_S =np.array(Arr_s)
from multiprocessing import Pool
from multiprocessing import Lock
# from moveaway_h import funct #due to OS constraints, implemented in a separate script
# from moveaway_h import publishing

def _init_s(Arr_sh):
    global Arr_S  
    Arr_S = Arr_sh


def funct(a):
    alpha_sample = 0.5
    alpha_threshold = 0.1
    r_nearest = 2
    reward_for_closeness = 10
    v = [0,0]
    
    n = len(Arr_S)
    
    for i in range(n):
        a_i = a - Arr_S[i]
        mod_a_i = np.linalg.norm(a_i)
        if (mod_a_i == 0) :
            continue
        if (mod_a_i >= r_nearest) :
            a_i = a_i/mod_a_i
            a_i = a_i * r_nearest
            v = v + a_i
            continue
        a_i = a_i/mod_a_i
        a_i = a_i * reward_for_closeness * r_nearest
        v = v + a_i
    mod_v = np.linalg.norm(v)
    if  mod_v < alpha_sample:
	    if mod_v < alpha_threshold :
	        return a
	    return a+v
    v_h = v/mod_v
    v_h = v_h * alpha_sample
    return a+v_h

# def check(a):
#     a = Arr_S(a[2]-1)

def publishing(a):
    
    if a[3]==0:
        return
    from geometry_msgs.msg import Pose2D
    import rospy
    #rate = rospy.Rate(10)
    d_pose = Pose2D()
    d_pose.x = a[0]
    d_pose.y = a[1]
    d_pose.theta = 0
    #temp_str = 'dest_pub_'+str(int(a[2]))
    #rospy.init_node(temp_str,anonymous = True)
    temp_str = 'robot'+str(int(a[2]))+'/d_pose'
    dest_pose_p = rospy.Publisher(temp_str,Pose2D,queue_size=10)
    rospy.loginfo('name: %s', dest_pose_p.name)
    
#    check(a)
    while not rospy.is_shutdown():
	time.sleep(0.5)
        connections = dest_pose_p.get_num_connections()
        rospy.loginfo('connections: %d', connections)
        
	
        if connections > 0:
            dest_pose_p.publish(d_pose)
            break
	
    global d
    d = [0,0]
    def get_source(data):
        global d
        d = np.array([data.x,data.y])

    temp_str = 'robot'+str(int(a[2]))+'/s_pose'

    source_pose_s = rospy.Subscriber(temp_str,Pose2D,get_source)

    while not rospy.is_shutdown():
        d_h = d - np.array([a[0],a[1]])
        if (np.linalg.norm(d_h) <= 0.1) :
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
    
  
    while not rospy.is_shutdown():
    
	    Arr_sh = np.array(source.Arr_s)
	    
	    

	    pool1 = Pool(initializer = _init_s, initargs= (Arr_sh,))
	    Arr_D = pool1.map(funct,Arr_sh)
	    pool1.close()
	    pool1.join()
            print Arr_D
	    
	    pool2 = Pool(initializer=boundary_conditions,initargs=(-5,5,-5,5))
	    check = pool2.map(check_for_boundary,Arr_D)
	    pool2.close()
	    pool2.join()
	    print check
	    if sum(check) == 0 :
	        break
	   # pool3 = Pool()
	    Arr_D_h = np.column_stack((Arr_D,np.arange(1,len(Arr_D)+1),check))
	    print Arr_D_h
        
        # multiprocessing.Pool.map not very convenient for publishing as ros is 
        # not having enough time to create the publishers and connect them to the subscribers
	    threads = []
	    for i in range(source.n):
		time.sleep(0.1)
	        thread = Thread(target = publishing, args = (Arr_D_h[i],)) 
	        threads.append(thread)
	        thread.start()

	    for thread in threads:
		    thread.join()
	    
	    #t2 = Thread(target = publishing, args = (Arr_D_h[1],))
	    #t2.start()

	    #t.join()
	    #publishing(Arr_D_h[0])
	    #publishing(Arr_D_h[1])

	    #pool3.map(publishing,Arr_D_h)
	    #pool3.close()
	    #pool3.join()

    rospy.signal_shutdown('reached the boundaries')
    

if __name__ == '__main__':
    rospy.init_node('source_pub')
    
    import source
    print source.n
    t = Thread(target = source.main)
    t.start()
    time.sleep(3)
    main()
    t.join()
    
























