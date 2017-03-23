#!/usr/bin/env python


import numpy as np
import rospy
#from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats
from neural_network.msg import LearningLabels
from geometry_msgs.msg import Twist

def init():
    rospy.init_node('network_command')
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, test_callback)

    rospy.loginfo('starting---------------')
    rospy.spin()
    

def test_callback(data_input):
   
    pub = rospy.Publisher('network_command', LearningLabels, queue_size=10)
    # construct the output command
    msg=LearningLabels()
    if data_input.angular.z==0 :
        msg.labels=np.array([0,0], dtype=np.float32)
        
    elif data_input.angular.z>0 :
        msg.labels=np.array([data_input.angular.z,0], dtype=np.float32)
        
    elif data_input.angular.z<0 :
        msg.labels=np.array([0,-data_input.angular.z], dtype=np.float32)
    #we need to add timestamps to the data
    msg.header.stamp=rospy.Time.now() 

    pub.publish(msg)


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
