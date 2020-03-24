import numpy as np
from numpy import deg2rad
from math import sin, cos, atan2, sqrt

'''
Uses the Haversine formula to calculate the dist between two points on Earth

Taken from https://www.movable-type.co.uk/scripts/latlong.html
'''

def calc_dist_lat_lon(lat1, lon1, lat2, lon2):
	'''
	Input: lat and lon are original coordinates (degrees in decimal)
		lat1 (float): lat from location 1
		lon1 (float): lon from location 1

		lat2 (float): lat from location 2
		lon2 (float): lon from location 2
	
	Output:
		dist (float): km distance from location 1 to location 2
	'''

	##assert that the values are valid within limits

	if not ((0 <= abs(lat1) <= 90) and (0 <= abs(lat2) <= 90)):
		raise Exception ("INVALID LAT INPUT")
	if not ((0 <= abs(lon1) <= 180) and (0 <= abs(lon2) <= 180)):
		raise Exception ("INVALID LON INPUT")

	R = 6371e3 ##meters

	##convert lat and lon to radians
	lat1_rad = deg2rad(lat1)
	lon1_rad = deg2rad(lon1)

	lat2_rad = deg2rad(lat2)
	lon2_rad = deg2rad(lon2)

	diff_lat = lat2_rad - lat1_rad
	diff_lon = lon2_rad - lon1_rad

	a = (sin(diff_lat/2))**2 + cos(lat1_rad) * cos(lat2_rad) * (sin(diff_lon/2))**2
	c = 2*atan2(sqrt(a), sqrt(1-a))

	dist = R*c
	return (dist)/1000