#!/usr/bin/env python


import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution
from pyNN.utility import Timer
import matplotlib.pyplot as plt
import pylab
import numpy as np
from scipy import signal


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

def network():
    rospy.init_node('simple_network_node')
    rate = rospy.Rate(10) # 10hz
    #rospy.Subscriber("camera/image_processed", Image, test_callback)
    #rospy.Subscriber("camera/rgb/image_raw", Image, test_callback)
    rospy.Subscriber('camera/image_compressed', Image, test_callback)
    #rospy.Subscriber("/chatter", String, callback)
    rospy.Subscriber("/test_image", Image, test_callback)

    rospy.loginfo('starting---------------')
    rospy.spin()
    #while True:
    #    rospy.loginfo_throttle(10, "This message will print every 10 seconds")

def gaussian_convolution(spikes,dt):
    #----------- works only after the simulation has run; not online!!!!!!!!
    kernel_size = 10
    gaussian_kernel = signal.gaussian(kernel_size, std=2)
    scaling_factor = 1/np.sum(gaussian_kernel)*1/dt
    gauss_rate = np.convolve(spikes,gaussian_kernel,mode='same')*scaling_factor
    mean_rate = np.mean(gauss_rate)
    return mean_rate

def test_callback(data_input):
    global message
    message = data_input.data
    rospy.loginfo('=====received data %r', message)

    msg_list = list(message)

    msg_list= [int(msg.encode('hex'),16) for msg in message]
    #
    #for i in
    #msg_list = int(message.encode('hex'),16)

    #print('============= Received image data.',message)
    rospy.loginfo('=====received data %r', msg_list[1])
    timer = Timer()
    dt = 0.1
    p.setup(timestep=dt) # 0.1ms


    #input = p.Population(1, p.SpikeSourceArray, {'spike_times': [[0,3,6]]}, label='input')

    n_input_neurons = len(msg_list)  #/4
    rospy.loginfo('====length of input image %r', n_input_neurons)

    input = p.Population(n_input_neurons, p.SpikeSourcePoisson, {'rate':msg_list})

    pop_1 = p.Population(1,p.IF_curr_exp, {}, label="pop_1")

    stat_syn = p.StaticSynapse(weight =50.0, delay=1)
    input_proj = p.Projection(input, pop_1, p.AllToAllConnector(),synapse_type=stat_syn, receptor_type='excitatory')

    pop_1.record(['v','spikes'])
    p.run(10)
    pop_1_data= pop_1.get_data()

    spikes = pop_1_data.segments[0].spiketrains[0]
    mean_rate = int(gaussian_convolution(spikes,dt))
    rospy.loginfo('=====mean_rate %r', mean_rate) # mean_rate = 64
    rate_command = mean_rate
    # rate coding of the spike train

    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    # construct the output command
    command = Twist()
    command.linear.x = np.abs((int(mean_rate)%10-4))*10+0.24 #0.24
    command.angular.z = -(int(mean_rate)%10-4.5)*10
	#int(mean_rate)%100/100.*np.pi-np.pi/2
    pub.publish(command)

    rospy.loginfo('=====send command %r', command.angular.y)


    fig_settings = {
        'lines.linewidth': 0.5,
        'axes.linewidth': 0.5,
        'axes.labelsize': 'small',
        'legend.fontsize': 'small',
        'font.size': 8
    }
    plt.rcParams.update(fig_settings)
    fig1=plt.figure(1, figsize=(6,8))

    def plot_spiketrains(segment):
        for spiketrain in segment.spiketrains:
            y = np.ones_like(spiketrain) * spiketrain.annotations['source_id']
            plt.plot(spiketrain, y, '.')
            plt.ylabel(segment.name)
            plt.setp(plt.gca().get_xticklabels(), visible=False)

    def plot_signal(signal, index, colour='b'):
        label = "Neuron %d" % signal.annotations['source_ids'][index]
        plt.plot(signal.times, signal[:, index], colour, label=label)
        plt.ylabel("%s (%s)" % (signal.name, signal.units._dimensionality.string))
        plt.setp(plt.gca().get_xticklabels(), visible=False)
        plt.legend()

    print("now plotting the network---------------")
    rospy.loginfo('--------now plotting---------------')
    n_panels = sum(a.shape[1] for a in pop_1_data.segments[0].analogsignalarrays) + 2
    plt.subplot(n_panels, 1, 1)
    plot_spiketrains(pop_1_data.segments[0])
    panel = 3
    for array in pop_1_data.segments[0].analogsignalarrays:
        for i in range(array.shape[1]):
            plt.subplot(n_panels, 1, panel)
            plot_signal(array, i, colour='bg'[panel%2])
            panel += 1
    plt.xlabel("time (%s)" % array.times.units._dimensionality.string)
    plt.setp(plt.gca().get_xticklabels(), visible=True)#

    #plt.show()
    #fig1.show()
    #plt.savefig("~/Spiking-Neural-Networks-on-Robotino/network_output.jpg")



if __name__ == '__main__':
    try:
        network()
    except rospy.ROSInterruptException:
        pass
