import numpy as np
import math

##Uses the Haversine formula to calculate the dist between two points on Earth
##Input: lat and lon are in original coordinates (degrees in decimal)
##Output: Distance in km from point to point

##Taken from https://www.movable-type.co.uk/scripts/latlong.html
def calc_dist_lat_lon(lat1, lon1, lat2, lon2):

	##assert that the values are valid within limits

	R = 6371e3 ##meters

	##convert lat and lon to radians
	lat1_rad = np.deg2rad(lat1)
	lon1_rad = np.deg2rad(lon1)

	lat2_rad = np.deg2rad(lat2)
	lon2_rad = np.deg2rad(lon2)

	diff_lat = lat2_rad - lat1_rad
	diff_lon = lon2_rad - lon1_rad

	a = (math.sin(diff_lat/2))**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * (math.sin(diff_lon/2))**2
	c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))

	dist = R*c

	return (dist)/1000