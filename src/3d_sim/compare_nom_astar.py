from math import atan2, sin, cos, acos
from copy import deepcopy
from datetime import datetime

import numpy as np
import shapely.geometry as sg
from shapely.ops import nearest_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from trash_utils.Env import Env
from trash_utils.UUV import UUV
from trash_utils.finder_utils import (grid_data, fix_waypoint_edges)
from trash_utils.trash_lib import ( init_hotspots, 
                                    visualize_trash_flow,
                                    update_trash_pos,
                                    visualize_trash_step)
from trash_utils.cc_utils import calc_mowing_lawn, search_for_trash
from trash_utils.fourD_utils import (cost_to_waypoint_v1, 
                                     follow_path_waypoints, 
                                     calculate_nominal_path_short,
                                     follow_path_order)

from hs4dpp_main_utils import calc_heuristic_denom, calc_all_headings
from hs4dpp_path_utils import calc_ball_cost, find_optimal_path_nrmpc

import pdb

'''
Does both nominal and 4D A* solution to compare results

'''

class Compare_Nom_AStar():

    def __init__(self):
        self.env = Env(20, 20) ##This contains (ufunc, vfunc, width, height, u_boost, v_boost)
        self.num_hotspots = 6

        self.uuv = UUV()
        
        #For 4D path planning
        self.desired_speed = 2.5722     ##meters/second (5 knots), 
        self.astar_goal_dist = 20
        # self.astar_goal_dist = 10
        self.uuv_end_threshold = 10
        # self.max_num_epochs = 15000
        self.max_num_epochs = 45000

        self.nom_goal_dist = 50

        self.heuristic_denom = calc_heuristic_denom(self.env, self.desired_speed)
        self.base_h_angle = 45

        self.savetime = datetime.now().strftime('%Y%m%d%H%M%S')



    def compare(self, ball_cart_pos, path_order, hotspot_dict):
        '''
        Calculates the paths to compare to the nominal solution by:
        - Calculating closest points between two hotspots
        - Calling find_optimal_path_nrmpc to find the path between the two points

        Path to cover each hotspot is a mowing the lawn pattern

        This is an apples to apples comparison between the two algorithms
        The time to search the hotspot is added to both nom and astar costs
        However, we use the astar total time to determine at what time the paths are executed
        That way, we can compare the cost of traveling the path at the same times

        Inputs:
            hotspot_dict (Dict)        : key = hotspot id number (1,2,..., num_hotspots)
                                        values = latest pos of all the trash in the hotspot
            wpt_spacing (int)          : distance (meters) between the nominal waypoints
            heuristic_denom (float)    :  
            ball_cart_pos (np.ndarray) : 
            path_order (np.ndarray)    : 

        Returns:
            cc_paths (np.ndarray)  : waypoints to do complete coverage of the hotspot
            all_paths (np.ndarray) : Nominal waypoints to travel from hotspot to hotspot
                                     To access the path from hotspot 1 to 4 for example:
                                     all_paths[1][4]
            final_path (np.ndarray) : 

        TODO: handle return values here
        '''

        ##For each of these hotspots, 
        ##Calculate the path to each other 
        ## and the complete coverage algorithm to cover them
        ##TODO: update this with some other data structure/add the ones for keeping track of costs

        results_txt_file = open('compare_soln.txt', "a")
        results_txt_file.write("\n\n\nSAVETIME: {0}\n".format(self.savetime))
        results_txt_file.close()

        final_path_nom = []
        final_path_astar = []
        astar_total_energy_cost = 0.0
        astar_total_time_sec = 0.0
        astar_total_paper_cost = 0.0

        nom_total_energy_cost = 0.0
        nom_total_time_sec = 0.0
        nom_total_paper_cost = 0.0

        last_trash_update_sec = 0.0
        ##Calculate the paths in the order given
        for p_idx in range(len(path_order)-1):

            ##Update the trash positions up to this time
            if astar_total_time_sec > 0:
                print ("Updating trash positions")
                for diff_sec in range(int(last_trash_update_sec), int(astar_total_time_sec)):
                    diff_hrs = diff_sec/3600
                    hotspot_dict = update_trash_pos(hotspot_dict, diff_hrs, self.env)
                last_trash_update_sec = astar_total_time_sec
            print ("Finished updating trash pos")


            ## Get convex hull of each hotspot plus some buffer
            convexhull_a = sg.MultiPoint(hotspot_dict[path_order[p_idx]][-1][:, 0:2]).convex_hull
            buffer_a = convexhull_a.buffer(5)
            ax, ay = buffer_a.exterior.coords.xy
            [minx, miny, maxx, maxy] = buffer_a.bounds
            cc_y_lines = np.arange(miny, maxy, 5)

            ##Calculate the CC pattern on this hotspot
            cc_wpts = calc_mowing_lawn(buffer_a, cc_y_lines)
            cc_depths = self.env.dfunc(cc_wpts)
            cc_wpts = np.hstack((cc_wpts, cc_depths.reshape(-1,1)))
            cc_wpts = fix_waypoint_edges(cc_wpts, self.env)

            ##Calculate the cost of traveling this cc_path at this time
            ##Add results to both nominal and astar costs
            fig = plt.figure()
            # check_pos = deepcopy(self.uuv.pos)
            energy_cost, time_cost_sec, est_cost = search_for_trash(cc_wpts, 
                                                                    hotspot_dict, 
                                                                    self.uuv, 
                                                                    self.env, 
                                                                    self.desired_speed, 
                                                                    astar_total_time_sec,
                                                                    *[False])


            # self.uuv.pos = deepcopy(check_pos)

            # check_energy, check_time_sec, check_paper_cost = follow_path_order([cc_wpts], hotspot_dict, self.uuv, self.env, self.desired_speed, astar_total_time_sec, **{'uuv_path_state': 'searching'} )

            print ("\n\n\n SECTION 1 CHECK. FINISHED SEARCHING. NOM PATH AND CHECK COSTS SHOULD BE THE SAME VALUES.")
            print ("NOMINAL PATH COSTS")
            print ("nom_total_cost: ", energy_cost)
            print ("nom_time_sec  : ", time_cost_sec)
            print ("nom cost_eq   : ", est_cost)

            # print ("CHECK COSTS")
            # print ("check_energy  : ", check_energy)
            # print ("check time    : ", check_time_sec)
            # print ("check cost    : ", check_paper_cost)

            # print ("self uuv pos after double check: ", self.uuv.pos)

            # pdb.set_trace()

            '''
            CHECK RESULTS ABOVE

            '''

            nom_total_energy_cost += energy_cost
            nom_total_time_sec += time_cost_sec
            nom_total_paper_cost += est_cost
            final_path_nom.extend([cc_wpts])

            astar_total_energy_cost += energy_cost
            astar_total_time_sec += time_cost_sec
            astar_total_paper_cost += est_cost            
            final_path_astar.extend([cc_wpts])

            print ("total energy  : ", nom_total_energy_cost)
            print ("total time (s): ", nom_total_time_sec)
            print ("total cost eq : ", nom_total_paper_cost)
            print ("final_path    : ", final_path_nom)

            results_txt_file = open('compare_soln.txt', "a")
            results_txt_file.write("SEARCH FOR TRASH AT HOTSPOT " + str(path_order[p_idx]))
            results_txt_file.write("\nTotal energy: " + str(energy_cost))
            results_txt_file.write("\nTotal time (sec) : " + str(time_cost_sec))
            results_txt_file.write("\nTotal cost eq: " + str(est_cost))
            results_txt_file.write("\n")
            results_txt_file.close()




            ## Get the convex hull of the corresponding hotspot plus some buffer
            convexhull_b = sg.MultiPoint(hotspot_dict[path_order[p_idx+1]][-1][:, 0:2]).convex_hull
            buffer_b = convexhull_b.buffer(5)
            bx, by = buffer_b.exterior.coords.xy

            ##Calculate the closest points between the hotspots
            pt1, pt2 = nearest_points(buffer_a, buffer_b)
            pt1_depth = self.env.dfunc([pt1.x, pt1.y])[0]
            pt2_depth = self.env.dfunc([pt2.x, pt2.y])[0]
            pt1_3d = np.array([pt1.x, pt1.y, pt1_depth])
            pt2_3d = np.array([pt2.x, pt2.y, pt2_depth])

            last_uuv_pos = deepcopy(self.uuv.pos)


            ##Calculate the nominal path
            nom_path = calculate_nominal_path_short(pt1_3d, pt2_3d, 50, self.env)
            nom_energy, nom_time_sec, nom_est_cost, none = follow_path_waypoints(nom_path,
                                                                        self.uuv.pos,
                                                                        self.uuv,
                                                                        self.env,
                                                                        self.desired_speed,
                                                                        astar_total_time_sec,
                                                                        *[False])

            # self.uuv.pos = deepcopy(last_uuv_pos)
            # anom_energy, anom_time_sec, anom_paper_cost = follow_path_order([nom_path], hotspot_dict, self.uuv, self.env, self.desired_speed, astar_total_time_sec, **{'uuv_path_state': 'following'})
       
            # self.uuv.pos = deepcopy(last_uuv_pos)
            # cnom_energy, cnom_time_sec, cnom_paper_cost = follow_path_order([nom_path], hotspot_dict, self.uuv, self.env, self.desired_speed, nom_total_time_sec, **{'uuv_path_state': 'following'})

            print ("\n\n\n\nSECTION 2: CHECK NOMINAL PATH. 1 NOM PATH AND 2 ANOM PATH SHOULD HAVE THE SAME RESULTS. 3 CHECK NOM COSTS CAN BE DIFFERENT")

            print ("1 NOMINAL PATH COSTS")
            print ("nom_total_cost: ", nom_energy)
            print ("nom_time_sec  : ", nom_time_sec)
            print ("nom cost_eq   : ", nom_est_cost)

            # print ("2 ANOM costs")
            # print ("check_energy  : ", anom_energy)
            # print ("check time    : ", anom_time_sec)
            # print ("check cost    : ", anom_paper_cost)

            # print ("3 CHECK NOM costs")
            # print ("check_energy  : ", cnom_energy)
            # print ("check time    : ", cnom_time_sec)
            # print ("check cost    : ", cnom_paper_cost)

            # pdb.set_trace()            

            nom_total_energy_cost += nom_energy
            nom_total_time_sec += nom_time_sec
            nom_total_paper_cost += nom_est_cost
            final_path_nom.extend([nom_path])

            self.uuv.pos = deepcopy(last_uuv_pos)
            astr_eq_cost, astr_time_cost_sec, astr_energy_cost, astr_path = find_optimal_path_nrmpc(astar_total_time_sec, pt1_3d, pt2_3d, ball_cart_pos, self.uuv, self.env, self.heuristic_denom, self.desired_speed, self.astar_goal_dist, self.uuv_end_threshold, self.max_num_epochs, *[False])

            self.uuv.pos = deepcopy(last_uuv_pos)
            acheck_energy, acheck_time_sec, acheck_paper_cost = follow_path_order([astr_path], hotspot_dict, self.uuv, self.env, self.desired_speed, astar_total_time_sec, **{'uuv_path_state': 'following'})

            print ("\n\n\n\n SECTION 3: CHECKING ASTAR PATH COSTS. ASTAR AND CHECK COSTS SHOULD BE THE SAME")
            print ("ASTAR COSTS")
            print ("total energy  : ", astr_energy_cost)
            print ("total time (s): ", astr_time_cost_sec)
            print ("total cost eq : ", astr_eq_cost)

            print ("CHECK COSTS")
            print ("check_energy: ", acheck_energy)
            print ("check time  : ", acheck_time_sec)
            print ("check cost  : ", acheck_paper_cost)

            print ('END OF FOLLOWING PATH CHECK. ON TO SEARCHING NEXT HOTSPOT')

            astr_eq_cost = acheck_paper_cost
            astr_time_cost_sec = acheck_time_sec
            astr_energy_cost = acheck_energy

            ##Add these costs to the total costs
            # astar_total_energy_cost += astr_energy_cost
            # astar_total_time_sec += astr_time_cost_sec
            # astar_total_paper_cost += astr_eq_cost

            astar_total_energy_cost += acheck_energy
            astar_total_time_sec += acheck_time_sec
            astar_total_paper_cost += acheck_paper_cost
            final_path_astar.extend([astr_path])

            print ("NOMINAL PATH COSTS")
            print ("nom_total_cost: ", nom_energy)
            print ("nom_time_sec  : ", nom_time_sec)
            print ("nom cost_eq   : ", nom_est_cost)

            print ("ASTAR COSTS")
            print ("total energy  : ", astr_energy_cost)
            print ("total time (s): ", astr_time_cost_sec)
            print ("total cost eq : ", astr_eq_cost)

            # pdb.set_trace()





            plt.clf()
            fig = plt.figure()
            ax1 = fig.gca(projection='3d')
            ax1.plot([pt1_3d[0]], [pt1_3d[1]], [pt1_3d[2]], 'ko')
            ax1.plot([pt2_3d[0]], [pt2_3d[1]], [pt2_3d[2]], 'ko')
            ax1.text(pt1_3d[0], pt1_3d[1], pt1_3d[2], 'pt1')
            ax1.text(pt2_3d[0], pt2_3d[1], pt2_3d[2], 'pt2')
            ax1.plot(astr_path[:,0], astr_path[:,1], astr_path[:,2], 'r--')
            ax1.plot(astr_path[:,0], astr_path[:,1], astr_path[:,2], 'ro')
            ax1.plot(nom_path[:,0], nom_path[:,1], nom_path[:,2], 'b--')
            ax1.plot(nom_path[:,0], nom_path[:,1], nom_path[:,2], 'bo')
            ax1.set_title('{0}_{1}_path. \nEnergy cost: nom {2}, a* {3}. \nTime (sec) cost: nom {4} a* {5}'.format(path_order[p_idx], path_order[p_idx+1], nom_energy, astr_energy_cost, nom_time_sec, astr_time_cost_sec))
            plt.savefig('{0}_{1}_path_{2}'.format(path_order[p_idx], path_order[p_idx+1], self.savetime))
            # plt.show()


            results_txt_file = open('compare_soln.txt', "a")
            results_txt_file.write("\nHOTSPOT{0} TO HOTSPOT{1}\n".format(path_order[p_idx],
                                                                     path_order[p_idx+1]))
            results_txt_file.write("NOMINAL PATH COSTS\n")
            results_txt_file.write("nom_total_cost: " + str(nom_energy))
            results_txt_file.write("\nnom_time_sec  : " + str(nom_time_sec))
            results_txt_file.write("\nnom cost_eq   : " + str(nom_est_cost))

            results_txt_file.write("\nASTAR COSTS")
            results_txt_file.write("\ntotal energy  : " + str(astr_energy_cost))
            results_txt_file.write("\ntotal time (s): " + str(astr_time_cost_sec))
            results_txt_file.write("\ntotal cost eq : " + str(astr_eq_cost))
            results_txt_file.write("\n\n")
            results_txt_file.close()


        ## Run search pattern for the last hotspot in the order
        convexhull_a = sg.MultiPoint(hotspot_dict[path_order[p_idx+1]][-1][:, 0:2]).convex_hull
        buffer_a = convexhull_a.buffer(5)
        ax, ay = buffer_a.exterior.coords.xy
        [minx, miny, maxx, maxy] = buffer_a.bounds
        cc_y_lines = np.arange(miny, maxy, 5)

        ##Calculate the CC pattern on this hotspot
        cc_wpts = calc_mowing_lawn(buffer_a, cc_y_lines)
        cc_depths = self.env.dfunc(cc_wpts)
        cc_wpts = np.hstack((cc_wpts, cc_depths.reshape(-1,1)))
        cc_wpts = fix_waypoint_edges(cc_wpts, self.env)

        ##Calculate the cost of traveling this cc_path at this time
        # fig = plt.figure()
        energy_cost, time_cost_sec, est_cost = search_for_trash(cc_wpts, 
                                                                hotspot_dict, 
                                                                self.uuv, 
                                                                self.env, 
                                                                self.desired_speed, 
                                                                astar_total_time_sec,
                                                                *[False])

        nom_total_energy_cost += energy_cost
        nom_total_time_sec += time_cost_sec
        nom_total_paper_cost += est_cost
        final_path_nom.extend([cc_wpts])

        astar_total_energy_cost += energy_cost
        astar_total_time_sec += time_cost_sec
        astar_total_paper_cost += est_cost            
        final_path_astar.extend([cc_wpts])

        print ("total energy  : ", nom_total_energy_cost)
        print ("total time (s): ", nom_total_time_sec)
        print ("total cost eq : ", nom_total_paper_cost)
        print ("final_path    : ", final_path_nom)

        results_txt_file = open('compare_soln.txt', "a")
        results_txt_file.write("SEARCH FOR TRASH AT HOTSPOT " + str(path_order[p_idx+1]))
        results_txt_file.write("\nTotal energy: " + str(energy_cost))
        results_txt_file.write("\nTotal time (sec) : " + str(time_cost_sec))
        results_txt_file.write("\nTotal cost eq: " + str(est_cost))
        results_txt_file.write("\n")
        results_txt_file.close()


        ##print the final costs of the whole thing
        print ("TOTAL COSTS OVERALL")
        print ("TOTAL NOM COSTS")
        print ("Energy cost: ", nom_total_energy_cost)
        print ("Time (sec) : ", nom_total_time_sec)
        print ("Eq cost    : ", nom_total_paper_cost)

        print ("TOTAL ASTAR COSTS: ")
        print ("Energy cost: ", astar_total_energy_cost)
        print ("Time (sec) : ", astar_total_time_sec)
        print ("Eq cost    : ", astar_total_paper_cost)

        results_txt_file = open('compare_soln.txt', "a")
        results_txt_file.write("\nTOTAL COSTS OVERALL\n")
        results_txt_file.write("TOTAL NOM COSTS\n")
        results_txt_file.write("Energy cost: "+str(nom_total_energy_cost))
        results_txt_file.write("\nTime (sec) : "+str(nom_total_time_sec))
        results_txt_file.write("\nEq cost    : "+str(nom_total_paper_cost))

        results_txt_file.write("\nTOTAL ASTAR COSTS: \n")
        results_txt_file.write("Energy cost: "+str(astar_total_energy_cost))
        results_txt_file.write("\nTime (sec) : "+str(astar_total_time_sec))
        results_txt_file.write("\nEq cost    : "+str(astar_total_paper_cost))
        results_txt_file.write("\n\n")
        results_txt_file.close()

        ##Save all the path results
        with open('nom_final_path{0}.npy'.format(self.savetime), 'wb') as f:
            np.save(f, final_path_nom)
        with open('astar_final_path{0}.npy'.format(self.savetime), 'wb') as f:
            np.save(f, final_path_astar)


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
                                     trash_sigma, 20, self.env.dfunc)

        for xx in range(1000):
            hotspot_dict = update_trash_pos(hotspot_dict, 0, self.env)


        ball_cart_pos, ball_sphere_headings = calc_all_headings(self.base_h_angle)
        hotspot_order = [0, 1, 2, 3, 4, 5]

        self.compare(ball_cart_pos, hotspot_order, hotspot_dict)


CNA = Compare_Nom_AStar()
CNA.main()