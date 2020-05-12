#!/usr/bin/env python

import pickle
from math import atan2

import numpy as np
import rospy
import rospkg
import tf
from tf.transformations import euler_from_quaternion

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# from node_example.cfg import nodeExampleConfig as ConfigType

# Import gazebo message data types.
from gazebo_msgs.msg import ModelState, ModelStates

from trash_utils.finder_utils import get_current_block, get_depth_block

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
        self.prefix = rospy.get_param("~prefix", "can")
        self.rate = rospy.get_param("~rate", 1.0)

        self.u, self.v, self.uv, self.u_func, self.v_func, self.uv_func = get_current_block(bbox, current_filepath)
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
            if model_name.startswith(self.prefix):
                model_state = ModelState()
                model_state.model_name = model_name
                model_state.pose = model_pose
                model_state.twist = model_twist

                ##Broadcast the local frame to the world frame?
                # br = tf.TransformBroadcaster()
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

                try:
                    listener.waitForTransform(model_name, 'world', rospy.Time.now(), rospy.Duration(0.50))
                    (trans, rot) = listener.lookupTransform(model_name, 'world', rospy.Time(0))
                    print ("model name: ", model_name)
                    print ('rot: ', rot)
                    print ('trans: ', trans)

                    ##normalize quaternion
                    

                except:
                    continue
                
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

                ##Calculate transformation from global to local
                global_uv = np.array([u_current, v_current, 0]).reshape(-1, 1)
                local_change = rot*global_uv + trans

                print ("global_uv: ", global_uv)
                print ("local_uv:  ", local_change)

                # print ("-----------")
                # print ("can: ", model_name)
                # print ("location: ", model_state.pose.position)
                # print ("twist: ", model_state.twist.linear)
                # print ("u_current: ", u_current[0])
                # print ("v_current: ", v_current[0])
                # print ("uv: ", uv)
                # print ("angle: ", angle)

                # tf_listener = tf.TransformListener()


                ##Sets position of the model
                # model_state.pose.position.x = radius * np.cos(self.rate * t * 2 * np.pi)
                # model_state.pose.position.y = radius * np.sin(self.rate * t * 2 * np.pi)
                # print ("z pos before update: ", model_state.pose.position.z)

                # model_state.pose.position.x += u_current[0]
                # model_state.pose.position.y += v_current[0]

                # model_state.twist.linear.x = u_current[0]
                # model_state.twist.linear.y = v_current[0]
                # model_state.twist.linear.z = 0.0

                # model_state.twist.linear.x = uv
                # model_state.twist.angular.z = angle

                if model_state.pose.position.z > 0:
                    model_state.pose.position.z = -0.1

                # pdb.set_trace()

                # print ("model_state twist: ", model_state.twist)
                # pdb.set_trace()

                # print ("z pos after update : ", model_state.pose.position.z)

                ##Calculate what the depth of the can would be at that point:
                ##TODO: FIGURE OUT HOW TO CALCULATE DEPTH PROPERLY
                ##SOMETHING WRONG WITH THE DEPTH CALCULATION
                # z_pos = self.depth_func([model_state.pose.position.x, 
                #                          model_state.pose.position.y])

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
    place_bbox = np.array(config['coords_bbox'])



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