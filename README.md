# multi-robot-motion-detection

# Instructions:
    Create 2 Drive for ros: https://github.com/AutonomyLab/create_autonomy

    copy the launch file folder into your ros package.

    copy the source,waypoint and destination_robotX to scripts folder of your ros package and make them executable

    clone the create 2 drive to your workspace

    modify the launch files depending on the location of the scripts.

    destination_robotX.py has the destination coordinates for robotX. Modify it to specify it's path

    source.py updates the robotX/s_pose from gazebo.

    excecute the following on different terminals.

    roslaunch [your-package-name] robot_main.launch

    rosrun [your-package-name] source.py

    rosrun [your-package-name] destination_robotX.py


**waypoint.py :**

Creates an object of move2goal waits for the rospy to shutdown.  

move2goal:

·       s_pose, d_pose topics are created and subscribed to which are of Pose2D() type.

·       an update of d_pose triggers waypoint(s_pose,d_pose)

·       the namespaces for each bot are created in the launch files

**source.py :**

fetches data from gazebo and publishes it to respective s_pose for all the robots in the simulation  
note: this can be modified to launch in a specific namespace

**destination_robotX :**

contains the destination coordinates as a list  
subscribes -&gt; robotX/s_pose  
publishes -&gt; robotX/d_pose

drifting is tolerated by sampling the path into smaller lengths of d_sample.

# Move away from neighbours: 


**move_away_from_neighbors/source.py :**

similar to source.py described above but it also stores them in a global 2D array (Arr_s) and keeps updating them.

**Move away Algorithm:**

Arr = Arr_s

r = radius of consideration for individual robots.

Moveaway(robot) {

    for robot_i in Arr_s :

        if (distance(robot,robot_i) >= r):

            since robot_i is far; reduce the effect of it on deciding the direction the robot needs to move.

        else:

            since robot_i is too close; increase its effect in deciding the direction the robot needs to move.

    Based on the direction, calculate the resultant destination coordinates and return them.

}

**move_away_from_neighbors/destination.py:**

·       runs move_away_from_neighbors/source.py on a thread to access the updated array

·       parallelly, it implements the “moveaway algorithm” to figure out the destinations for the robots and                publishes them to their respective topics until they reach the boundaries.

 
