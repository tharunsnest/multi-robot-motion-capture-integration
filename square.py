#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate(pub):
    t = Twist()
    relative_angle = PI/2
    current_angle = 0
    angular_speed = 4.25
    t.angular.z = angular_speed
    t0 = rospy.Time.now().to_sec()
    while(current_angle < relative_angle):
        pub.publish(t)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)
    t.angular.z = 0
    pub.publish(t)



def move(pub):
    
    t = Twist()
    speed = 0.5
    distance = 0.5
    t.linear.x = speed
    
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < distance):
        #Publish the velocity
        pub.publish(t)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    t.linear.x = 0
    #Force the robot to stop
    pub.publish(t)

if __name__ == '__main__':
    try:
        rospy.init_node('driver', anonymous=True)
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        while not rospy.is_shutdown():
        #Testing our function
            for i in range(4):
                move(pub)
                rotate(pub)
    except rospy.ROSInterruptException: pass