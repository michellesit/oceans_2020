from trash_utils.finder_utils import load_currents, get_width, get_height

class Env():
	'''
	Contains all of necessary environment properties

	'''

	def __init__(self, uboost=1, vboost=1):
		self.place_bbox, self.ufunc, self.vfunc, self.dfunc = load_currents()
		self.width = get_width(self.place_bbox)
		self.height = get_height(self.place_bbox)
		self.max_depth = 50
		self.u_boost = uboost
		self.v_boost = vboost

		self.xbound = [-self.width/2, self.width/2]
		self.ybound = [-self.height/2, self.height/2]
		self.map_dim = [self.xbound, self.ybound]