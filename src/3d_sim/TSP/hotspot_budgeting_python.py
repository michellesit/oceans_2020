import math
import matplotlib.pyplot as plt

import numpy as np

np.random.seed(248745)


class Hotspot():
    def __init__(self, number, profits, area, uuv_obs_rate):
        self.number = number
        self.profits = profits
        self.area = area
        self.lam = uuv_obs_rate/area

if __name__ == '__main__':
	##import all hotspot to hotspot travel time over all time intervals
	# with open('./../santa_monica_energy_costs.npy', 'rb') as f:
	#     transit_times = np.load(f)

	##build all the hotspot data
	uuv_obs_rate = (5**2)*math.pi #arbitrarily selected observation rate
	all_hotspots = [Hotspot(0, np.random.randint(0,100), np.random.randint(500,2500), uuv_obs_rate),
	                Hotspot(1, np.random.randint(0,100), np.random.randint(500,2500), uuv_obs_rate),
	                Hotspot(2, np.random.randint(0,100), np.random.randint(500,2500), uuv_obs_rate),
	                Hotspot(3, np.random.randint(0,100), np.random.randint(500,2500), uuv_obs_rate),
	                Hotspot(4, np.random.randint(0,100), np.random.randint(500,2500), uuv_obs_rate),
	                Hotspot(5, np.random.randint(0,100), np.random.randint(500,2500), uuv_obs_rate)]

	# x = np.linspace(0, 150)
	x = np.arange(0, 150)
	lam = np.array([k.lam for k in all_hotspots])
	pi = np.array([k.profits for k in all_hotspots])
	# y = (1 - np.exp(-lam.reshape(-1,1)))*pi.reshape(-1,1)

	fig = plt.plot()	
	for i in range(len(all_hotspots)):
		y = (1 - np.exp(-lam[i] * x)) * pi[i]
		plt.plot(x, y, label=i)
	plt.legend()
	plt.show()


	# fig = plt.plot()
	# for i in range(len(y)):
		# plt.plot(x, y[i, :])
	# plt.show()