#!/usr/bin/env python

import pickle
from math import atan2

import numpy as np
import rospy
import rospkg
import tf
from tf.transformations import euler_from_quaternion, euler_matrix

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# from node_example.cfg import nodeExampleConfig as ConfigType

# Import gazebo message data types.
from gazebo_msgs.msg import ModelState, ModelStates

from trash_utils.finder_utils import get_current_block, get_depth_block, get_multi_current_block

import turtlesim.msg

import pdb

'''
Written by Ruffin White-Magner, add-ons by Michelle Sit
Sets the trash positions

'''

class Move_Cans(object):
    """Node example class."""

    def __init__(self, bbox, current_filepath, topo_filepath):
        """Read in parameters."""
        # Get the private namespace parameters from the parameter server:
        # Initialize from either command line or launch file.
        self.enable = rospy.get_param("~enable", True)
        self.prefix_can = rospy.get_param("~prefix", "can")
        self.prefix_rov = rospy.get_param("~prefix", "rexrov")
        self.rate = rospy.get_param("~rate", 1.0)

        self.u, self.v, self.uv, self.u_func, self.v_func, self.uv_func = get_current_block(bbox, current_filepath)
        # self.u_func, self.v_func, self.uv_func = get_multi_current_block(bbox, currents_path1, currents_path2)

        self.depth_func = get_depth_block(bbox, topo_filepath)

        
        # Create a dynamic reconfigure server.
#         self.server = DynamicReconfigureServer(ConfigType, self.reconfigure_cb)
                
        # Create a publisher and subscriber for gazebo.
        self.pub = rospy.Publisher(
            "/gazebo/set_model_state", ModelState,
            queue_size=10)
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates,
            self.move_cans_cb, queue_size=10)

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher."""
        self.sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates,
            self.move_cans_cb, queue_size=10)

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def move_cans_cb(self, model_states):
        """Call at a specified interval to publish message."""
        if not self.enable:
            return
        
        t = rospy.get_rostime().to_sec()

        br = tf.TransformBroadcaster()
        listener = tf.TransformListener()
        
        # Loop over model states
        for model_name, model_pose, model_twist in zip(
            model_states.name, model_states.pose, model_states.twist):
            # Filter via model names
            # if model_name.startswith(self.prefix_can) | model_name.startswith(self.prefix_rov):
            if model_name.startswith(self.prefix_rov):
                continue
                model_state = ModelState()
                model_state.model_name = model_name
                model_state.pose = model_pose
                model_state.twist = model_twist

                ##Broadcast the local frame to the world frame?
                br.sendTransform((model_state.pose.position.x,
                                  model_state.pose.position.y,
                                  model_state.pose.position.z),
                                 (model_state.pose.orientation.x, 
                                  model_state.pose.orientation.y,
                                  model_state.pose.orientation.z,
                                  model_state.pose.orientation.w),
                                 rospy.Time.now(),
                                 model_name,
                                 'world')

                # try:
                #     listener.waitForTransform(model_name, 'world', rospy.Time.now(), rospy.Duration(0.50))
                #     (q_trans, q_rot) = listener.lookupTransform(model_name, 'world', rospy.Time(0))
                #     # print ("model name: ", model_name)
                #     # print ('rot: ', q_rot)
                #     # print ('trans: ', q_trans)

                #     (e_roll, e_pitch, e_yaw) = euler_from_quaternion(q_rot) ## or rotation?
                #     h_rot = euler_matrix(e_roll, e_pitch, e_yaw)

                # except:
                #     print ("waiting for something")
                #     continue
                
                try:
                    listener.waitForTransform(model_name, 'world', rospy.Time.now(), rospy.Duration(0.50))
                    (q_trans, q_rot) = listener.lookupTransform(model_name, 'world', rospy.Time(0))
                    euler = euler_from_quaternion(q_rot)
                    h_rot = tf.transformations.compose_matrix(translate = q_trans, angles=euler)

                    # (e_roll, e_pitch, e_yaw) = euler_from_quaternion(q_rot) ## or rotation?
                    # h_rot = euler_matrix(e_roll, e_pitch, e_yaw)

                    # Compute next pose via clock
                    # radius = np.linalg.norm([
                    #         model_state.pose.position.x,
                    #         model_state.pose.position.y
                    #     ])

                    # print ("{0}: {1}".format(model_name, model_state.pose.position))

                    u_current = self.u_func([abs(model_state.pose.position.z), 
                                             model_state.pose.position.x,
                                             model_state.pose.position.y])
                    v_current = self.v_func([abs(model_state.pose.position.z), 
                                             model_state.pose.position.x,
                                             model_state.pose.position.y])

                    uv = np.linalg.norm([u_current, v_current])*10
                    angle = atan2(u_current, v_current)

                    # print ("q_trans: ", q_trans)
                    # print ("q_rot: ", q_rot)
                    # print ("h_rot: ", h_rot)

                    ##Calculate transformation from global to local
                    global_uv = np.array([u_current, v_current, 0, 1]).reshape(-1, 1)
                    local_change = h_rot*global_uv
                    print ("global_uv: ", global_uv)
                    # print ("LOCAL_CHANGE: ", local_change)

                    ##Apply changes to the UUV
                    model.state.pose.twist

                    if model_state.pose.position.z > 0:
                        model_state.pose.position.z = -0.1
                
                except:
                    continue

                # Publish to set model state
                self.pub.publish(model_state)


    def reconfigure_cb(self, config, dummy):
        """Create a callback function for the dynamic reconfigure server."""
        # Fill in local variables with values received from dynamic reconfigure
        # clients (typically the GUI).
        self.prefix = config["prefix"]
        self.rate = config["rate"]

        # Check to see if node should be started or stopped.
        if self.enable != config["enable"]:
            if config["enable"]:
                self.start()
            else:
                self.stop()
        self.enable = config["enable"]

        # Return the new variables.
        return config


# Main function.
def main():
    rospack = rospkg.RosPack()
    data_path = rospack.get_path('data_files')
    trash_finder_path = rospack.get_path('trash_finder')
    config = pickle.load(open(trash_finder_path + "/config/demo_configs.p", "rb"))

    topo_filepath = data_path + '/' + config['topo_file']
    current_filepath = data_path + '/' + config['current_file']
    currents1 = config['current_file1']
    currents_path1 = data_path + '/' + currents1
    currents2 = config['current_file2']
    currents_path2 = data_path + '/' + currents2
    # place_bbox = np.array(config['coords_bbox'])

    all_locations = pickle.load( open(trash_finder_path + "/config/locations.p", "rb"))
    place_bbox = all_locations["mission_bay_flatter_bbox"]



    # Initialize the node and name it.
    rospy.init_node("gazebo_orbits")
    # Go to class functions that do all the heavy lifting.
    try:
        Move_Cans(place_bbox, current_filepath, topo_filepath)
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()

# main()
if __name__ == "__main__":
    main()
