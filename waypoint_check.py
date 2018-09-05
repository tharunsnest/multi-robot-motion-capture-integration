import waypoint
dest= [(1,1)]
d_threshold= 0.01
x= waypoint.move2goal()
for i in range(len(dest)):
    
    p_f = waypoint.np.array(dest[i])
    p = waypoint.np.array(x.s_pose.x,x.s_pose.y) #get source

    x.d_pose.x = p_f[0]
    x.d_pose.y = p_f[1]

    x.dest_pose_pub.publish(x.d_pose)
    
    length=waypoint.np.linalg.norm(p-p_f) 

    while (abs(length) > d_threshold) :
        pass
        
    
    i = i+1