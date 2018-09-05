import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D
rospy.Subscriber('/gazebo/model_states',ModelStates,get_states)

def get_states(data):
    pass
    

