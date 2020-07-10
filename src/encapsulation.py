#!/usr/bin/env python

import rospy
import random
import numpy as np
import copy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from gazebo_msgs.msg import ModelStates
import readchar

## =========== User Setting ===============
keywords_of_object_to_encapsulate = ['xxxx', 'obstacle']
capsule_sizes = [[1.0, 1.0, 1.0], [0.15, 0.15, 0.7]]
robot_height = 1.0 # z-offset between the UR5 coordination and gazebo (See the length of 'stand' in 'rain_ur5_limited.urdf.xacro'). 
## ========================================

def pose_Gazebo2Rviz(pose_in_gazebo):
    pose = pose_in_gazebo
    pose.z += -robot_height
    return pose



def getCapsuleSize(object_name, keywords_of_object_to_encapsulate, capsule_sizes):
    __capsule_size = None # Initisalition

    for idx in range(0, len(keywords_of_object_to_encapsulate)):
        each_keyword = keywords_of_object_to_encapsulate[idx]
        if each_keyword in object_name:
            __capsule_size = capsule_sizes[idx]
    return __capsule_size


def genSafetyCapsule(env_data, keywords_of_object_to_encapsulate, capsule_sizes):
    __marker_array = MarkerArray()  # Initialisation
    __num_obstacle_identified = 0

    objects_in_env = env_data.name
    for idx in range(0, len(objects_in_env)):        
        __pose = pose_Gazebo2Rviz(env_data.pose[idx].position)
        each_object_name = objects_in_env[idx]
        __scale = getCapsuleSize(each_object_name, keywords_of_object_to_encapsulate, capsule_sizes)
        
        if __scale is not None:
            __marker = Marker(
                type=Marker.SPHERE,
                id=__num_obstacle_identified + 1000,
                lifetime=rospy.Duration(0.5),
                pose=Pose(__pose, Quaternion(0, 0, 0, 1)),
                scale=Vector3(__scale[0], __scale[1], __scale[2]), # Note: "scale" is a diameter.
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 0.8, 1.0, 0.5)) 


            __marker_array.markers.append(__marker)
            __num_obstacle_identified += 1


    return __marker_array




env_data = None  
def cb_env(data):
    global env_data
    env_data = data

def main():
    rospy.init_node('safety_capsule_generator')


    # Initisalisation for an publisher
    marker_array_publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
    rospy.sleep(0.5) 

    # Subscribing the Gazebo environment
    rospy.Subscriber('/gazebo/model_states', ModelStates, cb_env)

    print("Waiting for Gazebo Environment")
    while env_data is None: # Waiting for receiving the first message
      pass    
    print("Got Environment Information")



 


                                                                        
    
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        # Generate a capsule array accordingly (using MarkerArray) 
        __env_data = copy.deepcopy(env_data)
        capsules_array = genSafetyCapsule(__env_data, keywords_of_object_to_encapsulate = keywords_of_object_to_encapsulate, capsule_sizes = capsule_sizes)    

        # Publish the array     
        marker_array_publisher.publish(capsules_array)
        rate.sleep()



if __name__ == '__main__':
  main()