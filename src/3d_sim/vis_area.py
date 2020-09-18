import pickle
from math import atan2, sin, cos, acos

import rospkg
from scipy.io import netcdf
import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from collections import OrderedDict
from numpy.linalg import norm

from trash_utils.haversine_dist import haversine
from trash_utils.finder_utils import (get_multi_current_block,
									  get_current_block,
									  get_depth_block,
									  grid_data,
									  find_bound_idx,
									  xy_to_lat_lon,
									  wpts_to_yaml)

from trash_utils.trash_lib import trash_2d_gauss

import pdb

class Vis_Trash_Area():

	def __init__(self):
		self.place_bbox, self.ufunc, self.vfunc, self.dfunc, self.uvfunc = self.load_currents()
		self.width = haversine(self.place_bbox[0,0], self.place_bbox[0,1], self.place_bbox[1,0], self.place_bbox[1,1]) * 1000
		self.height = haversine(self.place_bbox[1,0], self.place_bbox[1,1], self.place_bbox[2,0], self.place_bbox[2,1]) * 1000

		self.num_hotspots = 6

		self.trash_x_centers = np.array([-250.98494701, -504.8406451, -132, 345, 876, 423]).reshape(-1,1)
		self.trash_y_centers = np.array([-508.96243035, -877.89326774, -687, 354, 120, 348]).reshape(-1,1)


	def load_currents(self):
		'''
		Initialize survey area bbox, u,v,depth functions

		Returns:
			place_bbox(np.ndarray):
			ufunc(function): time-dependent currents in u-direction. Inputs (time, depth, x,y)
			vfunc(function): time-dependent currents in v-direction. Inputs (time, depth, x,y)
			dfunc(function): returns bottom depth at that coordinate. Inputs (x,y)

		'''
		rospack = rospkg.RosPack()
		trash_finder_path = rospack.get_path("trash_finder")
		data_path = rospack.get_path("data_files")

		config = pickle.load(open(trash_finder_path + '/config/demo_configs.p', 'rb'))

		##hydrodynamic current
		current_path = data_path+'/ca_subCA_das_2020010615.nc'
		current_data = netcdf.NetCDFFile(current_path)

		currents1 = config['current_file1']
		currents_path1 = data_path + '/' + currents1
		currents2 = config['current_file2']
		currents_path2 = data_path + '/' + currents2
		depth = current_data.variables['depth'][:].copy()
		current_data.close()

		##bathymetry model
		topo = config['topo_file']
		topo_path = data_path + '/' + topo

		all_locations = pickle.load( open(trash_finder_path + "/config/locations.p", "rb"))
		place_bbox = all_locations["mission_bay_flatter_bbox"]

		place_bbox[:, 0] -= 

		print (place_bbox)

		ufunc, vfunc, uvfunc = get_multi_current_block(place_bbox, currents_path1, currents_path2)
		d_area, dfunc = get_depth_block(place_bbox, topo_path)

		return place_bbox, ufunc, vfunc, dfunc, uvfunc


	def vis_trash(self, width, height):
		sigma = [[0.5, 0.8], [0.8, 0.5]]
		num_soda_cans = 10
		trash_dict = {}

		# for hotspot_idx in range(self.num_hotspots):
		# 	hotspot_trash_coords = trash_2d_gauss([trash_x_centers[hotspot_idx][0], trash_y_centers[hotspot_idx][0]], sigma, num_soda_cans, self.dfunc)
		# 	trash_dict[hotspot_idx] = [hotspot_trash_coords]



	def vis_distances(self):
		##Plot the centers of the init trash clusters
		# fig = plt.figure()
		plt.plot(self.trash_x_centers, self.trash_y_centers, 'ko')

		trash_centers = np.array(zip(self.trash_x_centers, self.trash_y_centers))
		for t1 in range(self.num_hotspots):
			plt.text(self.trash_x_centers[t1]-10, self.trash_y_centers[t1]-10, '{0}'.format(t1))

			for t2 in range(self.num_hotspots):
				if t1 == t2 or t1>t2:
					continue

				random_y = np.random.randint(100)

				dist = abs(np.linalg.norm([trash_centers[t1] - trash_centers[t2]]))
				midpt = (trash_centers[t2] + trash_centers[t1])/2

				plt.plot([self.trash_x_centers[t1], self.trash_x_centers[t2]], [self.trash_y_centers[t1], self.trash_y_centers[t2]], 'b--')
				plt.text(midpt[0], midpt[1]+random_y, '{0} to {1} cost: {2}'.format(t1, t2, dist))

				# print ('{0} to {1} cost: {2}'.format(t1, t2, dist))

		# plt.show()


	def visualize_multi_current_and_distances(self):
		'''
		visualizes all pieces

		'''

		xwidth = [-self.width/2, self.width/2]
		yheight = [-self.height/2, self.height/2]

		depths = grid_data(self.dfunc, xwidth, yheight, 100, [], [])
		depths += 1
		depths_flat = depths.flatten()
		# time_range = range(0, 145, 3)
		# time_range = range(0, 24, 3)
		# time_range = range(24, 48, 3)
		# time_range = range(48, 72, 3)
		time_range = range(72, 96, 3)

		##init figure outside
		##8 figures for each day
		##6 days total

		##Visualize 2D
		w,h = np.meshgrid(np.linspace(-self.width/2, self.width/2, depths.shape[1], endpoint=True),
						  np.linspace(-self.height/2, self.height/2, depths.shape[0], endpoint=True))    

		fig = plt.figure(1)
		fig.suptitle('Current vector field over time (day4)')
		rows = 2
		cols = 4

		for t_step in range(len(time_range)):
			u = grid_data(self.ufunc, xwidth, yheight, 100, depths, [time_range[t_step]])
			v = grid_data(self.vfunc, xwidth, yheight, 100, depths, [time_range[t_step]])
			uv = grid_data(self.uvfunc, xwidth, yheight, 100, depths, [time_range[t_step]])

			vmin = np.min(uv)
			vmax = np.max(uv)

			# self.vis_distances()
			ax1 = fig.add_subplot(rows, cols, t_step+1)
			ax1.set_title("(time(hrs)={0})".format(time_range[t_step]))
			ax1.set_xlabel('latitude')
			ax1.set_ylabel('longitude')

			ax1.scatter(w,h, color='b', s=15)
			im1 = ax1.quiver(w,h, u, v, uv)
			# fig.colorbar(im1)
			# im1.set_clim(vmin, vmax)

			ax1.set_xlim([w[0][0]-0.01, w[0][-1]+0.01])
			ax1.set_ylim([h[0][0]-0.01, h[0][-1]+0.03])
			ax1.axis('equal')

			ax1.plot(self.trash_x_centers, self.trash_y_centers, 'ko')
			trash_centers = np.array(zip(self.trash_x_centers, self.trash_y_centers))
			for t1 in range(self.num_hotspots):
				ax1.text(self.trash_x_centers[t1]-10, self.trash_y_centers[t1]-10, '{0}'.format(t1))

				for t2 in range(self.num_hotspots):
					if t1 == t2 or t1>t2:
						continue

					dist = abs(np.linalg.norm([trash_centers[t1] - trash_centers[t2]]))
					ax1.plot([self.trash_x_centers[t1], self.trash_x_centers[t2]], [self.trash_y_centers[t1], self.trash_y_centers[t2]], 'b--')



		fig.colorbar(im1, orientation='horizontal')
		im1.set_clim(0, 0.15)
		plt.show()


VTA = Vis_Trash_Area()
VTA.visualize_multi_current_and_distances()