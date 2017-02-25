import pyNN.nest as p
from pyNN.random import NumpyRNG, RandomDistribution

import numpy as np
from scipy import signal
#from sklearn import linear_model

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

# sim_time in msec
def spike_mean_rate(spikes, sim_time):
	return len(spikes) / sim_time

def generate_testImage(direction):
	potential = 100
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
def generate_labeledImages(nr):
	labeledImages = []
	for i in range(nr/3):
		labeledImages.append((generate_testImage("left"), [1,0]))
		labeledImages.append((generate_testImage("middle"), [0,0]))
		labeledImages.append((generate_testImage("right"), [0,1]))


	return labeledImages

	
###### Parameters #######
seed=8658764

input_nr = 9
readout_nr = 2
reservoir_nr = 300

simulation_time = 200.0
dt = 1.0

p.setup(timestep=dt) # 0.1ms

######################


###### Neurons #######

input_neurons = p.Population(input_nr, p.SpikeSourcePoisson())
readout_neurons = p.Population(readout_nr, p.IF_curr_exp, {}, label="readout")

reservoir = p.Population(reservoir_nr,p.IF_curr_exp, {}, label="reservoir")
######################

###### Synapses #######

stat_syn_res = p.StaticSynapse(weight =5.0, delay=1)
stat_syn_input = p.StaticSynapse(weight =50.0, delay=1)
stat_syn_rout = p.StaticSynapse(weight =0.0, delay=1)

######################

###### Connections #######

pconn = 0.5      # sparse connection probability

rng = NumpyRNG()
gamma = RandomDistribution('gamma', (2.0, 0.3), rng=rng)
res_conn = p.FixedProbabilityConnector(pconn, rng=rng)

inp_conn = p.AllToAllConnector()
rout_conn = p.AllToAllConnector()

connections = {}
connections['r2r'] = p.Projection(reservoir, reservoir, res_conn,
                                synapse_type=stat_syn_res, receptor_type='excitatory')
#connections['r2r'].set(weight=20*gamma.next(reservoir_nr*reservoir_nr))

connections['inp2r'] = p.Projection(input_neurons, reservoir, inp_conn,
                                      synapse_type=stat_syn_input,receptor_type='excitatory')

#print(gamma.next(input_nr*reservoir_nr))
connections['inp2r'].set(weight=50*gamma.next(input_nr*reservoir_nr))

connections['r2rout'] = p.Projection(reservoir, readout_neurons, rout_conn,
                                      synapse_type=stat_syn_rout,receptor_type='excitatory')


######################

####### Feed images to network and record #######
images_nr = 9 # Must be a factor of 3
labeledImages = generate_labeledImages(images_nr)

input_neurons.record(['spikes'])
reservoir.record(['spikes'])
readout_neurons.record(['spikes'])
	
X = np.zeros( (images_nr,reservoir_nr) )
# yi, expected nr of spikes for output neurons 
y1 = [] #left images labels
y2 = [] #right images labels
i = 0
for labeledImage in labeledImages:
	print('Image')
	#print(labeledImage[0])
	#input_neurons.set(rate=labeledImage[0])
	input_neurons = p.Population(input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})

	p.run(simulation_time)

	reservoir_data = reservoir.get_data(clear=True)

	mean_rates = []
	for spiketrain in reservoir_data.segments[0].spiketrains:
		mean_rate = spike_mean_rate(spiketrain, simulation_time)
		mean_rates.append(mean_rate)

	X[i] = mean_rates

	y1.append(labeledImage[1][0])
	y2.append(labeledImage[1][1])

	i=i+1


print('Average spike matrix X')
#print(X)
print('y1')
#print(y1)
print('y2')
#print(y2)


######### Fit weights to each output neuron with linear regression ###########

#w1 =  np.linalg.lstsq(X,y1)[0].tolist()
w1 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(reservoir_nr), X.T.dot(y1))[0].tolist()

# The coefficients
print('Weights w1')
#print(w1)

#w2 =  np.linalg.lstsq(X,y2)[0].tolist()
w2 = np.linalg.lstsq(X.T.dot(X) + 0.1*np.identity(reservoir_nr), X.T.dot(y2))[0].tolist()

print('Weights w2')
#print(w2)

#####################



######### Test accuracy ###########

print("\nTesting accuracy\n")
print("Test Image")
labeledImage = labeledImages[0]
print(labeledImage[0])

#input_neurons.set(rate=image)
input_neurons = p.Population(input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})
p.run(simulation_time)

readout_neurons_data = readout_neurons.get_data(clear=True)
strains = readout_neurons_data.segments[0].spiketrains

print('Mean rate output neurons before change of weights')
print('(' + str(spike_mean_rate(strains[0], simulation_time)) + \
',' + str(spike_mean_rate(strains[1], simulation_time)) + ')')


# Connection['r2rout'] looks like
# [ [r0, rout0, value], [r0, rout1, v], [r1, rout0, v] ... ]
w = []
for i in range(reservoir_nr):
	w.append(w1[i])
	w.append(w2[i])

connections['r2rout'].set(weight=w)

p.run(simulation_time)

readout_neurons_data = readout_neurons.get_data(clear=True)
strains = readout_neurons_data.segments[0].spiketrains

print('Mean rate output neurons after change of weights')
print('(' + str(spike_mean_rate(strains[0], simulation_time)) + \
',' + str(spike_mean_rate(strains[1], simulation_time)) + ')')



print("\nTest Image")
labeledImage = labeledImages[2]
print(labeledImage[0])

input_neurons = p.Population(input_nr, p.SpikeSourcePoisson, {'rate':labeledImage[0]})

p.run(simulation_time)

readout_neurons_data = readout_neurons.get_data(clear=True)
strains = readout_neurons_data.segments[0].spiketrains

print('Mean rate output neurons after change of weights')
print('(' + str(spike_mean_rate(strains[0], simulation_time)) + \
',' + str(spike_mean_rate(strains[1], simulation_time)) + ')')

