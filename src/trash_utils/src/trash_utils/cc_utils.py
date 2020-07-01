import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa
import matplotlib.pyplot as plt

from trash_utils.fourD_utils import follow_path_waypoints

import pdb

'''
All of the complete coverage algorithms to be shared across files

'''


def calc_mowing_lawn(bbox, y_bbox, start_end='right'):
    '''
    Calculate the intersection of the lines with the bbox
    Orders the waypoints according to either left/right start/end

    Input:
        bbox (sh.Polygon)      : some polygon to calculate the mowing lawn pattern
        y_bbox (np.ndarray)    : Array of heights that the lines are calculated at
                                 Each point in this array corresponds to a line.
        start_end (str)        : "right" or "left". 
                                 Determines which side the first waypoint comes from.
                                 This determines the ordering of all the following 
                                 waypoints.

    Output:
        waypoints (np.ndarray) : ordered waypoints on how to traverse this bbox
    '''
    all_intersections = []
    minx, miny, maxx, maxy = bbox.bounds
    for i in range(y_bbox.shape[0]):
        lines = sg.LineString([ [minx-abs(minx*0.4), y_bbox[i]],
                                [maxx+abs(maxx*0.4), y_bbox[i]] ])
        intersections = bbox.intersection(lines)

        if intersections.is_empty == False:
            ##TODO: sort the intersections so they are always left to right
            # print (zip(lines.xy[0], lines.xy[1]))
            # ziplines = zip(lines.xy[0], lines.xy[1])
            # npint = np.array(intersections)

            # plt.plot(bbox.exterior.coords.xy[0], bbox.exterior.coords.xy[1])
            # plt.plot(lines.xy[0], lines.xy[1])
            # if npint.ndim == 1:
            #   plt.scatter(npint[0], npint[1])
            # else:
            #   plt.scatter(npint[:,0], npint[:,1])
            # plt.show()
            # pdb.set_trace()

            all_intersections.append(np.array(intersections))

    ##TODO: Should add in a check here to make sure there are intersecting lines

    ##order the waypoints accordingly
    waypoints = []
    for i in range(len(all_intersections)):
        line_pts = all_intersections[i]

        if all_intersections[i].ndim == 1:
            waypoints.append(line_pts)
            # waypoints.extend((line_pts, line_pts))
            continue

        if start_end == "right":
            if i%2 == 0:
                waypoints.extend((line_pts[1], line_pts[0]))
            else:
                waypoints.extend((line_pts[0], line_pts[1]))

        elif start_end == "left":
            if i%2 == 0:
                waypoints.extend((line_pts[0], line_pts[1]))
            else:
                waypoints.extend((line_pts[1], line_pts[0]))

    return np.array(waypoints)


def search_for_trash(cc_path, trash_dict, uuv, env, desired_speed, *args):
    '''
    At each leg of the algorithm, 
    "searches" for trash by detecting distance to the trash position.
    Assumes the algorithm is the mowing the lawn pattern

    Inputs:
        cc_path (np.ndarray)  : waypoints to travel to
        trash_dict (Dict)     : all the trash hotspot latest position
        uuv (Obj)             : trash_utils/UUV.py to access (pos, max_thrust)
        env (Obj)             : trash_utils/Env.py object to access
                                (ufunc, vfunc, width, height, max_depth)
        desired_speed (float) : uuv speed (meters/sec) for each step

    *args in order:
        args1 (bool)          : True = Visualize trash and uuv movement
        args2 (matplotlib)    : plt.figure() or subplot to plot in

    Returns:
        energy_cost (float)   : amount of energy uuv needs to complete this path
        time_cost_sec (int)   : amount of time in seconds to complete this path
        eq_cost (float)       : cost calculated from Huynh, Dunbabin, Smith (ICRA 2015)
                                Equation 6 which takes energy and time into account

    '''

    energy_cost = 0
    time_cost_sec = 0
    total_cost = 0

    # uuv_pos = uuv.pos
    uuv_pos = np.copy(uuv.pos)
    all_trash_pos = np.array(trash_dict.values()).reshape(-1, 3)
    all_trash_pos[:,2] *= -1

    np_args = np.array(args).flatten()
    if np_args[0]:
        fig = np_args[1]
        ax1 = fig.add_subplot(122, projection='3d')
        ##Plot all the waypoints to travel to
        ax1.plot(cc_path[:,0], cc_path[:,1], cc_path[:,2], 'bo')
        ax1.plot(cc_path[:,0], cc_path[:,1], cc_path[:,2], 'b--')
        ##Final goal state as red dot
        ax1.plot([cc_path[-1, 0]], [cc_path[-1, 1]], [cc_path[-1, 2]], 'ro')
        ax1.text(cc_path[-1, 0], cc_path[-1, 1], cc_path[-1, 2], 'goal')
        ##All trash positions labeled with green dot
        ax1.plot(all_trash_pos[:,0], all_trash_pos[:,1], all_trash_pos[:,2], 'go')

        maxx = max(cc_path[:,0])
        minx = min(cc_path[:,0])
        miny = min(cc_path[:,1])
        maxy = max(cc_path[:,1])
        minz = min(cc_path[:,2])
        maxz = max(cc_path[:,2])

        ax1.set_xlim([minx-10, maxx+10])
        ax1.set_ylim([miny-10, maxy+10])
        ax1.set_zlim([minz-3, maxz+3])


    for pt_idx in range(len(cc_path)):
        ##Follow the waypoint and calculate the cost of getting to that point
        ##Check if any of the trash positions are near the uuv
        ##If yes, save the uuv position and trash position
        trash_info = {'trash_dict': all_trash_pos}
        energy, time_sec, eq_cost, all_detected_trash_pos = follow_path_waypoints(
                                                         np.array([cc_path[pt_idx]]),
                                                         uuv, 
                                                         env, 
                                                         1.5, 
                                                         False, **trash_info)
        energy_cost += energy
        time_cost_sec += time_sec
        total_cost += eq_cost

        ##TODO: Surface if trash detected during that leg
        ##Broadcast the position of the trash to the base station
        if len(all_detected_trash_pos) > 0:
            uuv_detected_pos = uuv.pos
            ##CARLOS: use detected_time_hrs to determine the u,v currents at that time
            detected_time_hrs = time_cost_sec / 3600.0

            if np_args[0]:
                ax1.plot(all_detected_trash_pos[:,0], 
                         all_detected_trash_pos[:,1],
                         all_detected_trash_pos[:,2], 'mo')
                for d in range(len(all_detected_trash_pos)):
                    ax1.text(all_detected_trash_pos[d,0],
                             all_detected_trash_pos[d,1],
                             all_detected_trash_pos[d,2], 'found!')

            ## Surface
            new_uuv_pos = [uuv_pos[0], uuv_pos[1], 0]
            ## energy, time_sec = cost_to_waypoint(uuv_pos, new_uuv_pos)
            # energy_cost += energy
            # time_cost_sec += time_sec
            # pdb.set_trace()
            uuv_pos = new_uuv_pos ##TODO: Do we need this?

            ##TODO CARLOS: This is where you'd predict the trash locations
            ##self.uuv in nom_simulation has a all_found_trash data structure. 
            ##Feel free to change it to whatever you need


        if np_args[0]:
            ax1.plot([uuv.pos[0]], [uuv.pos[1]], [uuv.pos[2]], 'ro')
            ax1.text(uuv.pos[0], uuv.pos[1], uuv.pos[2], 'uuv')
            plt.pause(0.05)

    if np_args[0]:
        plt.show()

    return energy_cost, time_cost_sec, eq_cost




'''
def simple_cc(search_bbox):
    rotation = np.arange(0, 360, 10)
    best_cost = 99999999999999999;
    for r in range(len(rotation)):
        rotated_bbox = sa.rotate(search_bbox, rotation[r]) ##Default, takes in rotation by degrees
        waypts = calc_mowing_lawn(rotated_bbox, y_bbox0, start_end="left")
        centered_waypoints = sa.rotate(sh.Polygon(waypts), -rotation[r])
        np_centered_waypoints = np.array(zip(centered_waypoints.exterior.coords.xy[0], 
                                 centered_waypoints.exterior.coords.xy[1]))
        cost = self.est_propulsion(np_centered_waypoints, best_cost, 
                                        est_uuv_pos=False, use_bound=True, 
                                        visualize=False)

        if cost < best_cost:
            best_cost = cost
            best_waypoints = np_centered_waypoints
            best_rotation = rotation[r]
            # best_time = cost

    return best_cost
'''