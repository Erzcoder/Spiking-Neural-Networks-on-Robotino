import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution

import numpy as np
from scipy import signal

from common import spike_mean_rate, generate_labeledImages, print_mean_spike_rate
from common import param

p.setup(timestep=param.dt)



###### Neurons #######

input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson())
readout_neurons = p.Population(param.readout_nr, p.IF_curr_exp, {}, label="readout")
reservoir = p.Population(param.reservoir_nr,p.IF_curr_exp, {}, label="reservoir")

######################

###### Synapses #######

stat_syn_res = p.StaticSynapse(weight =5.0, delay=1)
stat_syn_input = p.StaticSynapse(weight =50.0, delay=1)
stat_syn_rout = p.StaticSynapse(weight =0.0, delay=1)

######################

###### Connections #######

rng = NumpyRNG()
gamma = RandomDistribution('gamma', (2.0, 0.3), rng=rng)
res_conn = p.FixedProbabilityConnector(param.res_pconn, rng=rng)

inp_conn = p.AllToAllConnector()
rout_conn = p.AllToAllConnector()

connections = {}
connections['r2r'] = p.Projection(reservoir, reservoir, res_conn,
                                synapse_type=stat_syn_res, receptor_type='excitatory')
#connections['r2r'].set(weight=20*gamma.next(param.reservoir_nr*param.reservoir_nr))

connections['inp2r'] = p.Projection(input_neurons, reservoir, inp_conn,
                                      synapse_type=stat_syn_input,receptor_type='excitatory')

print(gamma.next(param.input_nr*param.reservoir_nr))
connections['inp2r'].set(weight=50*gamma.next(param.input_nr*param.reservoir_nr))

connections['r2rout'] = p.Projection(reservoir, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_rout,receptor_type='excitatory')



######################

####### Feed images to network and record #######
labeledImages = generate_labeledImages(param.images_nr)

input_neurons.record(['spikes'])
reservoir.record(['spikes'])
readout_neurons.record(['spikes'])

X = np.zeros( (param.images_nr,param.reservoir_nr) )
# Expected nr of spikes for readout neurons 
rout_left = [] #left images labels
rout_right = [] #right images labels
i = 0
for labeledImage in labeledImages:
	print('Image')
	print(labeledImage[0])
	input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})

	p.run(param.simulation_time)

	reservoir_data = reservoir.get_data(clear=True)

	mean_rates = []
	for spiketrain in reservoir_data.segments[0].spiketrains:
		mean_rate = spike_mean_rate(spiketrain, param.simulation_time)
		mean_rates.append(mean_rate)

	X[i] = mean_rates

	rout_left.append(labeledImage[1][0])
	rout_right.append(labeledImage[1][1])

	i=i+1


print('Average spike matrix X')
print(X)
print('Readout neuron left labels')
print(rout_left)
print('Readout neuron right labels')
print(rout_right)


######### Fit weights to each output neuron with linear regression ###########

w1 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(param.reservoir_nr), X.T.dot(rout_left))[0].tolist()

# The coefficients
print('Weights w1 reservoir - readout neuron left')
print(w1)

w2 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(param.reservoir_nr), X.T.dot(rout_right))[0].tolist()

print('Weights w2 reservoir - readout neuron right')
print(w2)

#####################



######### Test accuracy ###########

print("\nTesting accuracy\n")
print("Test Image")
image = labeledImages[0][0]
print(image)

#input_neurons.set(rate=image)
input_neurons = p.Population(param.input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})
p.run(param.simulation_time)

readout_neurons_data = readout_neurons.get_data(clear=True)
strains = readout_neurons_data.segments[0].spiketrains

print_mean_spike_rate('Mean rate output neurons before change of weights', 
					strains, param.simulation_time)


# Connection['r2rout'] looks like
# [ [r0, rout0, value], [r0, rout1, v], [r1, rout0, v] ... ]
w = []
for i in range(param.reservoir_nr):
	w.append(w1[i])
	w.append(w2[i])

connections['r2rout'].set(weight=w)

p.run(param.simulation_time)

readout_neurons_data = readout_neurons.get_data(clear=True)
strains = readout_neurons_data.segments[0].spiketrains

print_mean_spike_rate('Mean rate output neurons after change of weights', 
					strains, param.simulation_time)
