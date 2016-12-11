#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np


rospy.init_node('test_python_node')
rate = rospy.Rate(10) # 10hz
rospy.Subscriber("camera/image_processed", Image, callback)
pub = rospy.Publisher('neural_command', double, queue_size=10)



def callback(data):
    print('============= Received image data.')


