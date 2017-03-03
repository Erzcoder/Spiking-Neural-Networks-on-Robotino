

class param:
	seed = 8658764		# Seed for reproduction of random number
	input_nr = 9		# Number of input neurons
	readout_nr = 2		# Number of readout neurons
	reservoir_nr = 2	# Number of reservour neurons
	simulation_time = 200.0 # Simulation time for each input
	dt = 1				# Timestep in simulation
	res_pconn = 0.01	# sparse connection probability for reservoir
	images_nr = 3	 	# Number of training images. Must be a factor of 3



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

def spike_mean_rate(spikes, sim_time):
	return len(spikes) / sim_time

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
def generate_labeledImages(nr):
	labeledImages = []
	for i in range(nr/3):
		labeledImages.append((generate_testImage("right"), [0,10]))
		labeledImages.append((generate_testImage("middle"), [0,0]))
		labeledImages.append((generate_testImage("left"), [10,0]))


	return labeledImages

# title: title of result
# strains: spiking trains
# stime: simulation time
def print_mean_spike_rate(title, strains, stime):
	print(title)
	print('(' + str(spike_mean_rate(strains[0], stime)) + \
	',' + str(spike_mean_rate(strains[1], stime)) + ')')
