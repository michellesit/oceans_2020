import pickle
import numpy as np

'''
Pickles all the location data dictionary to be used elsewhere
All widths and height values are in kilometers
'''

locations = \
{
	"lajolla_cove_bbox" : np.array([[32.875, -117.289],
									[32.875, -117.266], 
									[32.857, -117.266], 
									[32.857, -117.289]]),
	"lajolla_cove_width" : 2.1479197192011843,
	"lajolla_cove_height" : 2.0015086796024986,

	"lajolla_cove_shore_bbox" : np.array([[32.871, -117.284],
										  [32.871, -117.263],
										  [32.851, -117.263],
										  [32.851, -117.284]]),
	"lajolla_cove_shore_width" : 1.9612325760293245,
	"lajolla_cove_shore_height" : 2.2238985328910363,

	"mission_bay_bbox" : np.array([[32.766, -117.287],
								   [32.766, -117.264],
								   [32.747, -117.264],
								   [32.747, -117.287]]),
	"mission_bay_width" : 2.150556797649856,
	"mission_bay_height" :2.1127036062467672,

	"mission_bay_flatter_bbox" : np.array([[32.766, -117.279],
										   [32.766, -117.256],
										   [32.747, -117.256],
										   [32.747, -117.279]]),
	"mission_bay_flatter_width" : 2.150556797649856,
	"mission_bay_flatter_height" : 2.1127036062467672,

	"san_fran_shoal_bbox" : np.array([[37.772, -123.113],
									 [37.772, -123.089],
									 [37.754, -123.089],
									 [37.754, -123.113]]),
	"san_fran_shoal_width" : 2.109468559942295,
	"san_fran_shoal_height" : 2.001508679601791,

	"san_fran_shoal_south" : np.array([[37.760, -123.094],
									  [37.760, -123.071],
									  [37.742, -123.071],
									  [37.742, -123.094]]),
	"san_fran_shoal_south" : 2.021902082193144,
	"san_fran_shoal_south" : 2.001508679601791

}

pickle.dump( locations, open("locations.p", "wb"))