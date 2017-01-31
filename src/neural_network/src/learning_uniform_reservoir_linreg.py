import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution

import numpy as np
from scipy import signal
from sklearn import linear_model

reservoir_exc=None
reservoir_inh=None

def gaussian_convolution(spikes,dt):
    #---- takes a spiketrain and the simulation time constant
    #     and computes the smoothed spike rate
    #-----works only after the simulation has run; not online!!!!!!!!
    kernel_size = 10
    gaussian_kernel = signal.gaussian(kernel_size, std=2)
    scaling_factor = 1/np.sum(gaussian_kernel)*1/dt
    gauss_rate = np.convolve(spikes,gaussian_kernel,mode='same')*scaling_factor
    mean_rate = np.mean(gauss_rate)
    return mean_rate




def generate_testImage(direction):
	potential = 10
	if direction=="left":
		return [potential,0,0,potential,0,0,potential,0,0]
	elif direction=='middle':
		return [0,potential,0,0,potential,0,0,potential,0]
	elif direction=='right':
		return [0,0,potential,0,0,potential,0,0,potential]
	else:
		return [0,0,0,0,0,0,0,0,0]

# Labeled image has the form (image, label)
# Label is a list [on1, on2], on# being the correct value for 
# the output neurons
def generate_labeledImages():
	labeledImages = []
	for i in range(10):
		labeledImages.append((generate_testImage("right"), [10,0]))
		labeledImages.append((generate_testImage("middle"), [0,0]))
		labeledImages.append((generate_testImage("left"), [0,10]))


	return labeledImages

	
###### Parameters #######
seed=8658764

input_nr = 9
readout_nr = 2
exc_nr = 20
inh_nr = 5

simulation_time = 50.0
dt = 1

p.setup(timestep=dt) # 0.1ms

######################


###### Neurons #######

input_neurons = p.Population(input_nr, p.SpikeSourcePoisson(rate=0))
readout_neurons = p.Population(readout_nr, p.IF_curr_exp, {}, label="readout")

reservoir_exc = p.Population(exc_nr,p.IF_curr_exp, {}, label="reservoir_exc")
reservoir_inh = p.Population(inh_nr,p.IF_curr_exp, {}, label="reservoir_inh")

######################

###### Synapses #######

stat_syn_exc = p.StaticSynapse(weight =5.0, delay=1)
stat_syn_inh = p.StaticSynapse(weight =20.0, delay=1)
stat_syn_input = p.StaticSynapse(weight =50.0, delay=1)

######################

###### Connections #######

pconn = 0.01      # sparse connection probability

rng = NumpyRNG(seed=seed)
exc_conn = p.FixedProbabilityConnector(pconn, rng=rng)
inh_conn = p.FixedProbabilityConnector(pconn, rng=rng)
inp_conn = p.AllToAllConnector()
rout_conn = p.AllToAllConnector()

connections = {}
connections['e2e'] = p.Projection(reservoir_exc, reservoir_exc, exc_conn,
                                synapse_type=stat_syn_exc, receptor_type='excitatory')
connections['e2i'] = p.Projection(reservoir_exc, reservoir_inh, exc_conn,
                                synapse_type=stat_syn_exc,receptor_type='excitatory')
connections['i2e'] = p.Projection(reservoir_inh, reservoir_exc, inh_conn,
                                synapse_type=stat_syn_inh,receptor_type='inhibitory')
connections['i2i'] = p.Projection(reservoir_inh, reservoir_inh, inh_conn,
                                synapse_type=stat_syn_inh,receptor_type='inhibitory')

connections['inp2e'] = p.Projection(input_neurons, reservoir_exc, inp_conn,
                                      synapse_type=stat_syn_input,receptor_type='excitatory')
connections['inp2i'] = p.Projection(input_neurons, reservoir_inh, inp_conn,
                                      synapse_type=stat_syn_input,receptor_type='excitatory')

connections['e2rout'] = p.Projection(reservoir_exc, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_exc,receptor_type='excitatory')
connections['i2rout'] = p.Projection(reservoir_inh, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_inh,receptor_type='inhibitory')


######################

####### Feed images to network and record #######

labeledImages = generate_labeledImages()

input_neurons.record(['spikes'])
reservoir_exc.record(['spikes'])
reservoir_inh.record(['spikes'])
readout_neurons.record(['spikes'])

# 30 images x 25 neurons, contains avarage nr of spikes for each neuron 
X = np.zeros( (30,25) )
# yi, expected nr of spikes for output neurons 
y1 = []
y2 = []
i = 0
for labeledImage in labeledImages:
	# ISSUE, can't feed in different images. 
	# WANTED: input_neurons.set(rate=labeledImage[0])
	print('Image')
	print(labeledImages[0][0])
	input_neurons.set(rate=labeledImages[0][0])

	p.run(simulation_time)



	reservoir_exc_data = reservoir_exc.get_data()
	reservoir_inh_data = reservoir_inh.get_data()
	input_neurons_data = input_neurons.get_data()

	#spikes_exc = reservoir_exc_data.segments[0].spiketrains[0]
	# spikes_inh = reservoir_inh_data.segments[0].spiketrains[0]
	#spikes_input = input_neurons_data.segments[0].spiketrains[0]
	#print('Input neruons spiking')
	#print(spikes_input)
	#print('Exc neruons spiking')
	#print(spikes_exc)

	mean_rates = []
	for spiketrain in reservoir_exc_data.segments[0].spiketrains:
		mean_rate = len(spiketrain)/simulation_time
		mean_rates.append(mean_rate)

	for spiketrain in reservoir_inh_data.segments[0].spiketrains:
		mean_rate = len(spiketrain)/simulation_time
		mean_rates.append(mean_rate)

	X[i] = mean_rates

	y1.append(labeledImage[1][0])
	y2.append(labeledImage[1][1])

	i=i+1


print('Average spike matrix X')
print(X)
print('y1')
print(y1)
print('y2')
print(y2)


######### Fit weights to each output neuron with linear regression ###########


regr = linear_model.LinearRegression()

regr.fit(X, y1)
w1 = regr.coef_

# The coefficients
print('Weights w1')
print(w1)

regr.fit(X, y2)
w2 = regr.coef_

print('Weights w2')
print(w2)

#####################



######### Test accuracy ###########

image = labeledImages[0][0]

input_neurons.set(rate=image)
p.run(simulation_time)

readout_neurons_data = readout_neurons.get_data()

print('Output neuron 1 before')
print(len(readout_neurons_data.segments[0].spiketrains[0]))

#print(len(w1[0:exc_nr]))
#print(len(connections['e2rout'].get('weight', format='list')))
#print('W before')
#print(connections['e2rout'].get('weight', format='list'))

# Connection['e2rout'] looks like [n1, n2, value]
w_exc = []
for i in range(exc_nr):
	w_exc.append(w1[i])
	w_exc.append(w2[i])


w_inh = []
for i in range(inh_nr):
	w_inh.append(w1[exc_nr + i])
	w_inh.append(w2[exc_nr + i])


print('Before set connection')
#connections['e2rout'].set(weight=w_exc)
connections['i2rout'].set(weight=w_inh)

#print('W after')
#print(connections['e2rout'].get('weight', format='list'))

p.run(simulation_time)

readout_neurons_data = readout_neurons.get_data()

print('Output neuron 1 after')
print(len(readout_neurons_data.segments[0].spiketrains[1]))

