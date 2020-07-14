import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.Env import Env
from trash_utils.UUV import UUV
from trash_utils.finder_utils import fix_waypoint_edges
from trash_utils.trash_lib import ( init_hotspots, 
                                    visualize_trash_flow,
                                    update_trash_pos,
                                    visualize_trash_step)
from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
from trash_utils.fourD_utils import cost_to_waypoint_v1, follow_path_waypoints

import pdb

'''
Deploy multiple trash hotspots in an environment
Calculate a nominal path in 3D that would go from spot to spot
Do complete coverage in those spots

This is the baseline for the experiment

'''

class Nom_Simulation():

    def __init__(self):
        self.Env = Env() ##This contains (ufunc, vfunc, width, height, u_boost, v_boost)
        self.num_hotspots = 6

        self.uuv = UUV()
        
        #For 4D path planning
        self.desired_speed = 2.5722     ##meters/second (5 knots)
        self.goal_dist = 50


    def calculate_nominal_paths(self, hotspot_dict, wpt_spacing):
        '''
        Calculates a path between each hotspot by
        - Calculating closest points between two hotspots
        - Breaking the path down into smaller pieces
        - Setting the waypoints to be those breakpoints with depth at the bottom of the map

        Path to cover each hotspot is a mowing the lawn pattern

        Inputs:
            hotspot_dict (Dict)    : key = hotspot id number (1,2,..., num_hotspots)
                                     values = latest pos of all the trash in the hotspot
            wpt_spacing (int)      : distance (meters) between the nominal waypoints

        Returns:
            cc_paths (np.ndarray)  : waypoints to do complete coverage of the hotspot
            all_paths (np.ndarray) : Nominal waypoints to travel from hotspot to hotspot
                                     To access the path from hotspot 1 to 4 for example:
                                     all_paths[1][4]

        '''

        ##For each of these hotspots, 
        ##Calculate the path to each other 
        ## and the complete coverage algorithm to cover them
        cc_paths = []
        all_paths = []
        for a_idx in range(self.num_hotspots):
            ## Get convex hull of each hotspot plus some buffer
            convexhull_a = sg.MultiPoint(hotspot_dict[a_idx][-1][:, 0:2]).convex_hull
            buffer_a = convexhull_a.buffer(5)
            ax, ay = buffer_a.exterior.coords.xy
            [minx, miny, maxx, maxy] = buffer_a.bounds
            cc_y_lines = np.arange(miny, maxy, 5)

            ##Calculate the CC pattern on this hotspot
            cc_wpts = calc_mowing_lawn(buffer_a, cc_y_lines, self.Env)
            cc_depths = self.Env.dfunc(cc_wpts)
            cc_paths.append(np.hstack((cc_wpts, cc_depths.reshape(-1,1))))
            cc_paths = fix_waypoint_edges(cc_wpts, self.Env)

            hotspot_paths = []
            for b_idx in range(self.num_hotspots):
                if b_idx == a_idx:
                    hotspot_paths.append([])
                    continue

                convexhull_b = sg.MultiPoint(hotspot_dict[b_idx][-1][:, 0:2]).convex_hull
                buffer_b = convexhull_b.buffer(5)
                bx, by = buffer_b.exterior.coords.xy

                ##Calculate the closest points between the hotspots
                pt1, pt2 = nearest_points(buffer_a, buffer_b)
                pt1_depth = self.Env.dfunc([pt1.x, pt1.y])[0]
                pt2_depth = self.Env.dfunc([pt2.x, pt2.y])[0]
                pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
                pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

                ##NOMINAL PATH
                ##Calculate the waypoints needed to traverse along the bottom of the map
                ##Split path into several points
                ##Get the depth for all of those points
                ##TODO? Smooth out the waypoints
                num_pts = abs(pt1.x - pt2.x)//wpt_spacing
                if num_pts == 0:
                    path = np.array([pt1_3d, pt2_3d])
                else:
                    waypts = np.array(zip(np.linspace(pt1.x, pt2.x, num_pts, endpoint=True), 
                                          np.linspace(pt1.y, pt2.y, num_pts, endpoint=True)))
                    waypts_depth = self.Env.dfunc(waypts)
                    path = np.hstack((waypts, waypts_depth.reshape(-1,1) ))

                hotspot_paths.append(path)
            all_paths.append(hotspot_paths)

        return cc_paths, all_paths


    def follow_path_order(self, nominal_path, trash_dict, vis_dash=True):
        '''
        
        Inputs:
            nominal_path (array) : Array of arrays. Each sub-array contains the waypoints 
                                   to do complete coverage within the hotspot
                                   or to travel from hotspot to hotspot
            trash_dict (Dict)    : key = hotspot id number (currrent 1,2,..., num_hotspots)
                                   values = latest positions of all the trash in the hotspot
            vis_dash (bool)      : True = visualize global path and 
                                   path for each hotspot searching or between hotspots

        Returns:
            total_trip_energy_cost (float) : total amount of energy uuv uses for mission
            total_trip_time_sec (int)      : total amount of time in sec to complete mission
            total_paper_cost (float)       : total cost calculated from Huynh, Dunbabin,
                                             Smith (ICRA 2015)
                                             Equation 6 which takes energy and time into 
                                             account

        '''

        ##Switch off between traveling to the cc areas and finding trash
        total_trip_energy_cost = 0
        total_trip_time_sec = 0
        total_paper_cost = 0

        vis_args = [False]
        uuv_path_state = 'searching'
        # uuv_path_state = 'path_following'

        for np_idx in range(len(nominal_path)):
            currently_following_path = nominal_path[np_idx]

            if vis_dash == True:
                fig = plt.figure()
                vis_args = [True, fig]

                ##Adds 2D overview of the map
                ax1 = fig.add_subplot(121)
                np_nominal_path = np.vstack(nominal_path)
                ##whole path
                ax1.plot(np_nominal_path[:,0], np_nominal_path[:,1], 'b--')
                ax1.plot(np_nominal_path[:,0], np_nominal_path[:,1], 'bo')
                ##What the uuv will tackle next
                ax1.plot(self.uuv.pos[0], self.uuv.pos[1], 'ro') ##uuv
                ax1.plot([self.uuv.pos[0], currently_following_path[0,0]],
                         [self.uuv.pos[1], currently_following_path[0,1]], 'k')
                ax1.plot(currently_following_path[:,0], currently_following_path[:,1], 'k')
                visualize_trash_step(trash_dict, [True, ax1])

            if uuv_path_state == 'path_following':
                print ("following")
                energy_cost, time_cost_sec, est_cost, none = follow_path_waypoints(
                                                             currently_following_path, 
                                                                        self.uuv.pos,
                                                                        self.uuv, 
                                                                        self.Env, 
                                                                        self.desired_speed,
                                                                        total_trip_time_sec,
                                                                        vis_args)

            if uuv_path_state == 'searching':
                print ('searching')
                energy_cost, time_cost_sec, est_cost = search_for_trash(
                                                       currently_following_path, 
                                                                        trash_dict, 
                                                                        self.uuv, 
                                                                        self.Env, 
                                                                        self.desired_speed,
                                                                        0,
                                                                        vis_args)

            print ("COST TO TRAVEL THIS LEG OF THE TRIP")
            print ("energy cost   : ", energy_cost)
            print ("time cost (s) : ", time_cost_sec)
            print ("est cost      : ", est_cost)
            print ("nominal_path: ", nominal_path[np_idx])

            ##Add up cost to travel this leg of the trip
            total_trip_energy_cost += energy_cost
            total_trip_time_sec += time_cost_sec
            total_paper_cost += est_cost

            if uuv_path_state == 'path_following':
                print ("switched to searching")
                uuv_path_state = 'searching'
            elif uuv_path_state == 'searching':
                print ("switched to following")
                uuv_path_state = 'path_following'

        return total_trip_energy_cost, total_trip_time_sec, total_paper_cost


    def main(self):
        ## Create hotspots of trash and (optinal) visualize
        trash_x_centers = np.array([-250.98494701, -504.8406451, \
                                    -132, 345, 876, 423]).reshape(-1,1)
        trash_y_centers = np.array([-508.96243035, -877.89326774, \
                                    -687, 354, 120, 348]).reshape(-1,1)
        ##Can specify trash distribution covariance for all hotspots:
        # trash_sigma = [[0.5, 0], [0, 0.1]]
        ##Use this to auto generate gaussian trash distributions:
        trash_sigma = [[], []]               
        hotspot_dict = init_hotspots(trash_x_centers, trash_y_centers,
                                     trash_sigma, 20, self.Env.dfunc)

        ##Step the hotspot dict some x num times into the future
        # fig = plt.figure()
        for xx in range(1000):
            hotspot_dict = update_trash_pos(hotspot_dict, 0, self.Env)
            ##Uncomment to keep map view standardized:
            # visualize_trash_step(hotspot_dict, [True, fig, self.Env.map_dim]) 

            ##Use this for unstandardized map view:
            # visualize_trash_step(hotspot_dict, [True, fig])
            # plt.pause(0.05)
        # plt.show()

        ##Calculate all complete coverage and inter-hotspot paths
        all_cc_paths, all_hotspot_paths = self.calculate_nominal_paths(hotspot_dict, 50)

        ## Arbitrarily selected order of hotspot traversal
        ## Get the path to follow
        hotspot_order = [0, 1, 2, 3, 4, 5]
        nominal_path = []
        for idx in range(len(hotspot_order)-1):
            nominal_path.extend([all_cc_paths[hotspot_order[idx]], 
                                 all_hotspot_paths[hotspot_order[idx]]\
                                                  [hotspot_order[idx+1]]])
        nominal_path.append(all_cc_paths[5])

        ##Execute the path
        total_energy_cost, total_time_sec, total_paper_cost = self.follow_path_order(
                                                                nominal_path,
                                                                hotspot_dict, 0)

        print ("FINAL COST VALUES:")
        print ("total energy  : ", total_energy_cost)
        print ("total time (s): ", total_time_sec)
        print ("total cost eq : ", total_paper_cost)

if __name__ == '__main__':
    HS = Nom_Simulation()
    HS.main()