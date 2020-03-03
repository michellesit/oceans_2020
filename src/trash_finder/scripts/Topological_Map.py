import numpy as np
from scipy.io import netcdf
import matplotlib.pyplot as plt


##read in the depth map
##Get the coordinates of the area of interest

class Topological_Map():

	def get_map_contours(self):
		socal = netcdf.NetCDFFile('./southern_calif_crm_v1.nc')
		xog = socal.variables['x'][:]
		yog = socal.variables['y'][:]
		zog = socal.variables['z'][:]

		xfilter = np.argwhere((xog <= -117.343) & (xog >= -117.543))
		xclip = xog[xfilter].flatten()

		yfilter = np.argwhere((yog>=32.66) & (yog<= 32.838))
		yclip = yog[yfilter].flatten()

		x,y = np.meshgrid(xclip, yclip)
		z1 = zog[yfilter[0]:yfilter[-1]+1, xfilter[0]:xfilter[-1]+1]

		fig, ax = plt.subplots(figsize=(15,15))
		CS = ax.contour(x,y,z1)
		ax.clabel(CS, inline=1, fontsize=10)
		# plt.show()

		contour = CS.collections
		return contour
		
		# contour.get_paths()[0].vertices