#!/usr/bin/env python


import numpy as np
import rospy
import message_filters

from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats
from neural_network.msg import LearningLabels
from sensor_msgs.msg import Image

from random import randint

nr_data=0
nr_train=18
nr_test=2
train_data = []
test_data = []

def init():
    rospy.init_node('data_generator')
    rate = rospy.Rate(10) # 10hz
    
    
    global image_sub
    image_sub = message_filters.Subscriber('camera/image_processed', Image)
    #image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
    global label_sub
    label_sub = message_filters.Subscriber('network_command', LearningLabels)

    ts = message_filters.TimeSynchronizer([image_sub, label_sub], 10)
    ts.registerCallback(callback)

    rospy.loginfo('starting---------------')
    rospy.spin()
    
# Labeled image has the form (image, label)
# Label is a list [on1, on2], on# being the correct value for 
# the output neurons

def callback(image,label):

    global nr_train
    global nr_test
    global nr_data
    global train_data
    global test_data

    nr_data=nr_data+1
    rospy.loginfo('Captured %d data.',nr_data)
    if randint(1,nr_train+nr_test)<=nr_test:
	test_data.append((image,label.labels))
    else:
        train_data.append((image,label.labels))

    if nr_data==nr_test+nr_train:
        global image_sub
        global label_sub
        image_sub.unregister()
        label_sub.unregister()
        use_data()






def use_data():
    i=randint(1,nr_train)
    rospy.loginfo('random data label %d: [%f,%f]',i,train_data[i][1][0],train_data[i][1][1])




class param:
	seed = 8658764			# Seed for reproduction of random number
	#rng = NumpyRNG()		# Use seed to reproduce 
	input_nr = 9			# Number of input neurons
	readout_nr = 2			# Number of readout neurons
	reservoir_nr = 10		# Number of reservour neurons
	res_exc_nr = 20			# Number of excitatory neurons
	res_inh_nr = 5			# Number of inhibitory neurons
	simulation_time = 200.0 # Simulation time for each input
	dt = 1					# Timestep in simulation
	res_pconn = 0.1		# sparse connection probability for reservoir
	images_train_nr = 9		# Number of training images to train with, 
							# Must be a factor of 3
	images_test_nr = 9  	# Number of test images
	#images_train = generate_labeledImages(images_train_nr)
	#images_test = generate_labeledImages(images_test_nr)

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass





