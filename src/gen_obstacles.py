#!/usr/bin/env python

import rospy
import random

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import readchar



def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)



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
      
def main():
  rospy.init_node('rviz_marker_array_publihser')

  # Initisalisation for an publisher
  marker_array_publisher = rospy.Publisher('visualization_marker_array', MarkerArray)
  rospy.sleep(0.5) 

  # Marker Array setting
  radius = 0.1
  num_obstacles = 10 
  range_obstacles = (0.3, 1.0, -0.1, 0.1, 0.0, 0.7) # It should include (x_min, x_max, y_min, y_max, z_min, z_max)
 
  # User Interface setting
  pos_stride = 0.015
  obstacle_id = 0

  # Generate a marker array accordingly
  sphere_array = gen_multiple_spheres_in_rviz(radius, num_obstacles, range_obstacles)  

  # Add a bottom plane
  sphere_array.markers.append(Marker(
                  type=Marker.SPHERE,
                  id=1001,
                  # lifetime=rospy.Duration(1.5),
                  pose=Pose(Point(0, 0, -0.1), Quaternion(0, 0, 0, 1)),
                  scale=Vector3(2*100, 2*100, 2*0.05), # Note: "scale" is a diameter.
                  header=Header(frame_id='base_link'),
                  color=ColorRGBA(0.0, 0.0, 1.0, 0.1)) )

  # Publish the array
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    marker_array_publisher.publish(sphere_array)
    key = readchar.readkey()
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
    elif key == '\x03':
        break

    
    rate.sleep()

  ## When this node is closed
  sphere_array = gen_multiple_spheres_in_rviz(radius, num_obstacles, range_obstacles, lifetime=rospy.Duration(0.5)) 
  marker_array_publisher.publish(sphere_array)
  marker_array_publisher.publish(MarkerArray())
 
  

if __name__ == '__main__':
  main()