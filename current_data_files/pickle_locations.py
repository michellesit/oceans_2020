import pickle
import rospkg
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
	"san_fran_shoal_width" : 2.021902082193144,
	"san_fran_shoal_height" : 2.001508679601791,

	"santa_monica_basin" : np.array([[33.798, -118.924],
								     [33.798, -118.805],
								     [33.691, -118.805],
								     [33.691, -118.924]]),
	"santa_monica_width" : 10.99600592490177,
	"santa_monica_height": 11.897857150967893,

	"san_pedro_basin" : np.array([[33.566, -118.458],
								  [33.566, -118.355],
								  [33.484, -118.355],
								  [33.484, -118.458]]),
	"san_pedro_width" : 9.543270468701273,
	"san_pedro_height" : 9.117983984854098,

	"catalina_basin" : np.array([[33.198, -118.480],
								 [33.198, -118.351],
								 [33.106, -118.351],
								 [33.106, -118.480]]),
	"catalina_width" : 12.00294249291961,
	"catalina_height" : 10.229933251299615,

	"san_nicolas": np.array([[33.517, -119.900],
                            [33.517, -119.786],
                            [33.428, -119.786],
                            [33.428, -119.900]]),
	"san_nicolas_width" :  10.568444607997142,
	"san_nicolas_height":  9.896348471366104

}

rospack = rospkg.RosPack()
trash_finder_path = rospack.get_path('trash_finder')
pickle.dump( locations, open(trash_finder_path+"/config/locations.p", "wb"))