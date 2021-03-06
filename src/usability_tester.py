#!/usr/bin/env python

import rospy
import random
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA, Float64MultiArray
from relaxed_ik.msg import EEPoseGoals

from threading import Thread
import copy

from datetime import datetime
import os


# ============ User-defined parameters ==============
# === Number of Tests
num_test = 30
goal_radius = 0.1 # (m)
dist_from_obstacles = 0.2 # (m)
# === Target Setting
range_target = (0.3, 0.6, -0.4, 0.4, 0.1, 0.6) # (x_min, x_max, y_min, y_max, z_min, z_max)
# === Random Obstacles Information Setting
radius = 0.1
num_obstacles = 5 
range_obstacles = (0.2, 0.7, -0.5, 0.5, 0.0, 0.7) # It should include (x_min, x_max, y_min, y_max, z_min, z_max)

# ===================================================



ee_pose_now = []
def cb_ee_pose_now(data):
    global ee_pose_now
    ee_pose_now = np.array([data.ee_poses[0].position.x, data.ee_poses[0].position.y, data.ee_poses[0].position.z])

min_dist_to_obs_now = []
def cb_min_dist_to_obs(data):
    global min_dist_to_obs_now
    min_dist_to_obs_now = []
    for a in data.data:
        min_dist_to_obs_now.append(a) 


def gen_multiple_spheres_in_rviz(radius, num_obstacles, range_obstacles, lifetime = rospy.Duration(0)):
      
      
  __marker_array = MarkerArray()   

  for __marker_id in range(0, num_obstacles):

    __random_pose = Point(random.uniform(range_obstacles[0], range_obstacles[1]), random.uniform(range_obstacles[2], range_obstacles[3]), random.uniform(range_obstacles[4], range_obstacles[5]))  


    
    __marker = Marker(
                  type=Marker.SPHERE,
                  id=__marker_id + 1000,
                  lifetime=lifetime,
                  pose=Pose(__random_pose, Quaternion(0, 0, 0, 1)),
                  scale=Vector3(2*radius, 2*radius, 2*radius), # Note: "scale" is a diameter.
                  header=Header(frame_id='base_link'),
                  color=ColorRGBA(0.0, 0.8, 1.0, 0.35)) 
     
    
    __marker_array.markers.append(__marker)

  return __marker_array 

def get_distance_btw_Points(point_1, point_2):
    dist = np.linalg.norm(np.array([point_1.x, point_1.y, point_1.z]) - np.array([point_2.x, point_2.y, point_2.z]))
    return dist
    

def get_target(range_target, obstacle_array, lifetime = rospy.Duration(1)):
    __marker_array = MarkerArray()   

    min_dist = 0
    while min_dist < dist_from_obstacles:
        __random_pose = Point(random.uniform(range_target[0], range_target[1]), random.uniform(range_target[2], range_target[3]), random.uniform(range_target[4], range_target[5]))  
        
        # Check if the randomised target pose is too much near to obstacles
        dist_list = []
        for obstacle_id in range(0, len(obstacle_array.markers)):
            __obstacle_pose = obstacle_array.markers[obstacle_id].pose.position
            __dist = get_distance_btw_Points(__obstacle_pose,__random_pose)
            dist_list.append(__dist)

        min_dist = min(dist_list)
        if min_dist < dist_from_obstacles:
            print("The random target is too close to obstacles\n")
     
    __marker = Marker(
                  type=Marker.CUBE,
                  id=0,
                  lifetime=lifetime,
                  pose=Pose(__random_pose, Quaternion(0, 0, 0, 1)),
                  scale=Vector3(goal_radius, goal_radius, goal_radius), # Note: "scale" is a diameter.
                  header=Header(frame_id='base_link'),
                  color=ColorRGBA(0.8, 0.0, 0.0, 0.7)) 

    __marker_array.markers.append(__marker)

    return __marker_array

def get_fileobject():
    time_0 = rospy.get_rostime()
    # Output file initialisation
    output_filename = "Usability_" + str(int(time_0.to_time())) + ".txt"
    f = open(output_filename, "w")
    now = datetime.now()
    # Textual month, day and year	
    f.write("Generated by usability_tester.py (" + str(now)+ ")\n")  
    f.write("Time_completion\tMinimum_distance\tOptimal_path_distance\n")     
    return f

success_flag = 1
def main():
  rospy.init_node('usability_tester')

  # Initisalisation for publishers (obstalce positions) and subscribers (end effector position; minimum distances from any closest obstacles and the end effector)
  obstacle_publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
  target_publisher = rospy.Publisher('target_marker', MarkerArray)  
  rospy.Subscriber('/relaxed_ik/ee_pose_now', EEPoseGoals, cb_ee_pose_now)
  rospy.Subscriber('/relaxed_ik/min_dist_to_obs', Float64MultiArray, cb_min_dist_to_obs)     
  

  print("Waiting for End Effector Position")
  while len(ee_pose_now) == 0: # Waiting for receiving the first message from /relaxed_ik/ee_pose_now
    pass    
  print("Got End Effector Position")

  rospy.sleep(0.5) 

  # Initialisation
  test_epoch = 0
  target = None
  obstacle_array = None
  optimal_distance = None
  min_dist_to_obs = 1.0
  success_flag = 1
  rate = rospy.Rate(10)      

  _fileobject = get_fileobject()

  while not rospy.is_shutdown():
    # For new test epoch: new target and new environment generation 
    if success_flag == 1:
        test_epoch += 1

        candidate_min_dist_to_obs = 0.0
        desired_distance = 0.12
        while candidate_min_dist_to_obs < desired_distance:
            obstacle_array = gen_multiple_spheres_in_rviz(radius, num_obstacles, range_obstacles)
            obstacle_publisher.publish(obstacle_array)
            rospy.sleep(0.05)
            # Check the minimum distance
            _min_dist_to_obs = copy.deepcopy(min_dist_to_obs_now)
            candidate_min_dist_to_obs = min(_min_dist_to_obs)   
            if candidate_min_dist_to_obs < desired_distance:
                print("Some obstalces are too close")             

        target = get_target(range_target, obstacle_array)
        min_dist_to_obs = 1.0

        _target_pose = np.array([target.markers[0].pose.position.x, target.markers[0].pose.position.y, target.markers[0].pose.position.z])        
        _ee_pose = copy.deepcopy(ee_pose_now)
        optimal_distance = np.linalg.norm(_ee_pose - _target_pose)        

        time_0 = rospy.get_rostime() 
        success_flag = 0
    
    if test_epoch > num_test: # Complete the test
        break

    # Publish the target
    target_publisher.publish(target)

    # Obstacle Random Move and Publish
    # obstacle_array = random_move_obstacles(obstacle_array) 
    obstacle_publisher.publish(obstacle_array)   

    # Check the minimum distance
    if len(min_dist_to_obs_now) is not 0:  
        _min_dist_to_obs = copy.deepcopy(min_dist_to_obs_now)
        candidate_min_dist_to_obs = min(_min_dist_to_obs)
        if min_dist_to_obs > candidate_min_dist_to_obs:
            min_dist_to_obs = candidate_min_dist_to_obs

    # Check the task completion
    _target_pose = np.array([target.markers[0].pose.position.x, target.markers[0].pose.position.y, target.markers[0].pose.position.z])
    _ee_pose = copy.deepcopy(ee_pose_now)
    _distance_to_target = np.linalg.norm(_ee_pose - _target_pose)
    
    time_now = rospy.get_rostime()
    time_spent = (time_now - time_0).to_sec()
    if time_spent > 400.0:
        print("Too long time")
        success_flag = 1
        test_epoch = test_epoch - 1

    if _distance_to_target < goal_radius:
        success_flag = 1
        time_now = rospy.get_rostime()
        time_completion = (time_now - time_0).to_sec()
        print("Epoch = ", str(test_epoch), " Success: Reached out the target!")
        print("time_completion: ", time_completion)
        print("min_dist_to_obs: ", min_dist_to_obs)
        _fileobject.write(str(time_completion)+ "\t" +str(min_dist_to_obs) + "\t" + str(optimal_distance) + "\n")
        
    

    rate.sleep()


  ## When this node is closed
  _fileobject.close()
  obstacle_array = gen_multiple_spheres_in_rviz(radius, num_obstacles, range_obstacles, lifetime=rospy.Duration(0.5)) 
  obstacle_publisher.publish(obstacle_array)
  obstacle_publisher.publish(MarkerArray())

         
    
    

def random_move_obstacles(obstacle_array):        
    del_move = 0.001
    obstacle_allowance_range = [0.0, 1.0, -0.5, 0.5, 0.0, 1.0] # x_min, x_max, y_min, y_max, z_min, z_max

    for obstacle_id in range(0, len(obstacle_array.markers)):
        move = np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)])
        unit_move = move/np.linalg.norm(move)
        
        new_x = obstacle_array.markers[obstacle_id].pose.position.x + del_move*unit_move[0]         
        new_y = obstacle_array.markers[obstacle_id].pose.position.y + del_move*unit_move[1] 
        new_z = obstacle_array.markers[obstacle_id].pose.position.z + del_move*unit_move[2] 

        if new_x >= obstacle_allowance_range[0] and new_x <= obstacle_allowance_range[1]:
            obstacle_array.markers[obstacle_id].pose.position.x = new_x
        if new_y >= obstacle_allowance_range[2] and new_y <= obstacle_allowance_range[3]:
            obstacle_array.markers[obstacle_id].pose.position.y = new_y
        if new_z >= obstacle_allowance_range[4] and new_z <= obstacle_allowance_range[5]:
            obstacle_array.markers[obstacle_id].pose.position.z = new_z                        

    return obstacle_array

 

if __name__ == '__main__':
  main()