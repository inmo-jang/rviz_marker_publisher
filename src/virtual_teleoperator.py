#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, Vector3


      
############ User Setting ##############
pub_rate = 250
xyzrpy = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]    # (m/s) XYZ velocity // (rad) Amplitudes of Roll Pitch Yaw in End Effector Frame
moving_period = 4     # Repeatative moving period (sec)


def main():
  rospy.init_node('virtual_teleoperator')

  # Initisalisation for an publisher
  cmd_publisher = rospy.Publisher('unity/twist1', Twist)
  rospy.sleep(0.5) 

  # Set a scenario of the virtual teloperator movement (Note that the values are relative to the previous point)
  waypoints = []
  waypoints.append((0.5, 0.0, 0.0))
  waypoints.append((-0.5, 0.0, 0.0))
  
  print("Waiting for ROS Time")
  while rospy.get_time() == 0: # Waiting for receiving the first message from /clock
    pass
  time_0 = rospy.get_rostime()
  print("Got ROS Time")

  # Publish the virtual user commands
  rate = rospy.Rate(pub_rate)

  while not rospy.is_shutdown():
    time_now = rospy.get_rostime()
    time_step = (time_now - time_0).to_sec()
    x = xyzrpy[0]*np.sin(2*np.pi*time_step/moving_period)
    y = xyzrpy[1]*np.sin(2*np.pi*time_step/moving_period)
    z = xyzrpy[2]*np.sin(2*np.pi*time_step/moving_period)
    
    palm_del_distance = Twist(linear = Vector3(x, y, z))


    cmd_publisher.publish(palm_del_distance)
    rate.sleep()

  
 
  

if __name__ == '__main__':
  main()