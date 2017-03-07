#!/usr/bin/env python
"""
@author Nicolas Berberich, Elie Aljalbout
@date   14.01.2016

This is a simple network to demonstate the closed loop between
a network running on the SpiNNaker board and a robot simulated in gazebo.
"""

from pyNN.random import NumpyRNG, RandomDistribution
from pyNN.utility import Timer
import matplotlib.pyplot as plt
import pylab
import numpy as np
from scipy import signal
import rospy
from std_msgs.msg import Int64



import spynnaker.pyNN as pynn
from ros_spinnaker_interface import ROS_Spinnaker_Interface
from ros_spinnaker_interface import SpikeSourcePoisson
from ros_spinnaker_interface import SpikeSinkSmoothing
# import transfer_functions as tf

def network():
    # simulator setup
    dt = 0.1                     # simulation timestep in ms
    simulation_time = 50000      # in ms
    pynn.setup(timestep=dt, min_delay=dt, max_delay=2.0*dt) # ensure real-time execution

    n_input = 3

    ros_interface = ROS_Spinnaker_Interface(
        n_neurons_source=n_input,                 # number of neurons of the injector population
        Spike_Source_Class=SpikeSourcePoisson,   # the transfer function ROS Input -> Spikes you want to use.
        Spike_Sink_Class=SpikeSinkSmoothing,     # the transfer function Spikes -> ROS Output you want to use.
                                                    # You can choose from the transfer_functions module
                                                    # or write one yourself.
        output_population=pop,                      # the pynn population you wish to receive the
                                                    # live spikes from.
        ros_topic_send='to_spinnaker',              # the ROS topic used for the incoming ROS values.
        ros_topic_recv='from_spinnaker',            # the ROS topic used for the outgoing ROS values.
        clk_rate=1000,                              # mainloop clock (update) rate in Hz.
        ros_output_rate=10) # number of ROS messages send out per second.

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
    msg_list = list(message)

    msg_list[1] = int(message[1].encode('hex'),16)
    #for i in
    #msg_list = int(message.encode('hex'),16)

    #print('============= Received image data.',message)
    rospy.loginfo('=====received data %r', msg_list[1])
    timer = Timer()
    dt = 0.1
    p.setup(timestep=dt) # 0.1ms


    pop_1 = p.Population(1,p.IF_curr_exp, {}, label="pop_1")
    #input = p.Population(1, p.SpikeSourceArray, {'spike_times': [[0,3,6]]}, label='input')
    input = p.Population(1, p.SpikeSourcePoisson, {'rate':msg_list[1]})
    stat_syn = p.StaticSynapse(weight =50.0, delay=1)
    input_proj = p.Projection(input, pop_1, p.OneToOneConnector(),synapse_type=stat_syn, receptor_type='excitatory')

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
    command.linear.x = rate_command*0.02
    command.angular.z = rate_command/50000.
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
