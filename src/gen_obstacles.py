#!/usr/bin/env python

import rospy
import random
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import readchar


mode = 'TrajectoryTest' 
# mode = 'Manual' 
# mode = 'RandomMove'
obstacle_id = 0 # Intialisation for "Manual" mode

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
                  color=ColorRGBA(0.0, 0.8, 1.0, 0.5)) 
     
    
    __marker_array.markers.append(__marker)

  return __marker_array 

def gen_long_sphere(lifetime = rospy.Duration(0)):
      
      
    __marker_array = MarkerArray()   

    __pose = Point(0.3, 0.0, 0.3)  
    __radius = [0.1, 100.0, 0.1]

    
    __marker = Marker(
                  type=Marker.SPHERE,
                  id=1000,
                  lifetime=lifetime,
                  pose=Pose(__pose, Quaternion(0, 0, 0, 1)),
                  scale=Vector3(2*__radius[0], 2*__radius[1], 2*__radius[2]), # Note: "scale" is a diameter.
                  header=Header(frame_id='base_link'),
                  color=ColorRGBA(0.0, 0.8, 1.0, 0.5)) 
     
    
    __marker_array.markers.append(__marker)

    return __marker_array 


def main():
  rospy.init_node('rviz_marker_array_publihser')

  # Initisalisation for an publisher
  marker_array_publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
  rospy.sleep(0.5) 

  # Marker Array setting
  radius = 0.1
  num_obstacles = 20 
#   range_obstacles = (0.3, 0.6, -0.05, 0.05, 0.0, 0.7) # It should include (x_min, x_max, y_min, y_max, z_min, z_max)
  range_obstacles = (-0.1, 0.9, 0.30, 0.50, 0.0, 1.0) # It should include (x_min, x_max, y_min, y_max, z_min, z_max)    


  # Generate a marker array accordingly
  sphere_array = gen_multiple_spheres_in_rviz(radius, num_obstacles, range_obstacles)  

  # Add a bottom plane
#   sphere_array.markers.append(Marker(
#                   type=Marker.SPHERE,
#                   id=2001,
#                   # lifetime=rospy.Duration(1.5),
#                   pose=Pose(Point(0, 0, -0.1), Quaternion(0, 0, 0, 1)),
#                   scale=Vector3(2*100, 2*100, 2*0.05), # Note: "scale" is a diameter.
#                   header=Header(frame_id='base_link'),
#                   color=ColorRGBA(0.0, 0.0, 1.0, 0.1)) )

  # Add a glove box
#   box_side_half = 0.2      
#   sphere_array.markers.append(Marker( # Right
#                   type=Marker.SPHERE,
#                   id=1003,
#                   # lifetime=rospy.Duration(1.5),
#                   pose=Pose(Point(0.0, -box_side_half, 0.0), Quaternion(0, 0, 0, 1)),
#                   scale=Vector3(2*100, 2*0.05, 2*100), # Note: "scale" is a diameter.
#                   header=Header(frame_id='base_link'),
#                   color=ColorRGBA(0.0, 1.0, 0.0, 0.1)) )   
                                                                                    

  # Publish the array
  rate = rospy.Rate(50) # 10hz
  while not rospy.is_shutdown():
    if mode == 'Manual':
        marker_array_publisher.publish(sphere_array)
        key = readchar.readkey()
        if key == '\x03':
            break        
        sphere_array = modify_sphere_array(sphere_array, key)

    elif mode == 'RandomMove':
        marker_array_publisher.publish(sphere_array)     
        sphere_array = random_move(sphere_array)

    elif mode == 'TrajectoryTest':
        sphere_array = gen_long_sphere()
        marker_array_publisher.publish(sphere_array)
    
    rate.sleep()

  ## When this node is closed
  sphere_array = gen_multiple_spheres_in_rviz(radius, num_obstacles, range_obstacles, lifetime=rospy.Duration(0.5)) 
  marker_array_publisher.publish(sphere_array)
  marker_array_publisher.publish(MarkerArray())
 


def random_move(sphere_array):        
    del_move = 0.001
    obstacle_allowance_range = [0.0, 1.0, -0.5, 0.5, 0.0, 1.0] # x_min, x_max, y_min, y_max, z_min, z_max

    for obstacle_id in range(0, len(sphere_array.markers)):
        move = np.array([random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0)])
        unit_move = move/np.linalg.norm(move)
        
        new_x = sphere_array.markers[obstacle_id].pose.position.x + del_move*unit_move[0]         
        new_y = sphere_array.markers[obstacle_id].pose.position.y + del_move*unit_move[1] 
        new_z = sphere_array.markers[obstacle_id].pose.position.z + del_move*unit_move[2] 

        if new_x >= obstacle_allowance_range[0] and new_x <= obstacle_allowance_range[1]:
            sphere_array.markers[obstacle_id].pose.position.x = new_x
        if new_y >= obstacle_allowance_range[2] and new_y <= obstacle_allowance_range[3]:
            sphere_array.markers[obstacle_id].pose.position.y = new_y
        if new_z >= obstacle_allowance_range[4] and new_z <= obstacle_allowance_range[5]:
            sphere_array.markers[obstacle_id].pose.position.z = new_z                        
    

    return sphere_array


def modify_sphere_array(sphere_array, key):        
        pos_stride = 0.015
        global obstacle_id

        if key == 'w':
            sphere_array.markers[obstacle_id].pose.position.x += pos_stride  
        elif key == 'x':
            sphere_array.markers[obstacle_id].pose.position.x -= pos_stride
        elif key == 'a':
            sphere_array.markers[obstacle_id].pose.position.y += pos_stride
        elif key == 'd':
            sphere_array.markers[obstacle_id].pose.position.y -= pos_stride
        elif key == 'q':
            sphere_array.markers[obstacle_id].pose.position.z += pos_stride
        elif key == 'z':
            sphere_array.markers[obstacle_id].pose.position.z -= pos_stride
        elif key == '0':
            obstacle_id = 0;
            print("Selected Obstacle", obstacle_id)
        elif key == '1':
            obstacle_id = 1;
            print("Selected Obstacle", obstacle_id)        
        elif key == '2':
            obstacle_id = 2;
            print("Selected Obstacle", obstacle_id)        
        elif key == '3':
            obstacle_id = 3;
            print("Selected Obstacle", obstacle_id)        
        elif key == '4':
            obstacle_id = 4;
            print("Selected Obstacle", obstacle_id)        
        elif key == '5':
            obstacle_id = 5;
            print("Selected Obstacle", obstacle_id)        
        elif key == '6':
            obstacle_id = 6;
            print("Selected Obstacle", obstacle_id)        
        elif key == '7':
            obstacle_id = 7;
            print("Selected Obstacle", obstacle_id)                                                                            
        elif key == '8':
            obstacle_id = 8;
            print("Selected Obstacle", obstacle_id)        
        elif key == '9':
            obstacle_id = 9;        
            print("Selected Obstacle", obstacle_id)        


        return sphere_array

if __name__ == '__main__':
  main()