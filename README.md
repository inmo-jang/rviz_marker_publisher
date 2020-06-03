# rviz_marker_publisher

This is a simple ROS publisher for MarkerArray in Rviz. The current version can publish multiple spherical markers. 


## (1) Obstacles Generation

### Step 1) Modify "Marker Array Setting" in `gen_obstacles.py`

- `radius`: the radius of each sphere
- `num_obstacles`: the number of spheres that you are going to visualise
- `range_obstacles`: the Cartesian range where the spheres will be uniform-randomly appeared. It should be such that  `(x_min, x_max, y_min, y_max, z_min, z_max)`

### Step 2) Run the following:

  ``` rosrun rviz_marker_publisher gen_obstacles.py```

The result MarkerArray will be published via `visualization_marker_array` topic. 


## (2) Virtual teleoprator

### Step 1) Modify user settings in `virtual_teleoprator.py`

- `xyzrpy`: (m/s) XYZ velocity // (rad) Amplitudes of Roll Pitch Yaw in End Effector Frame
- `moving_period`: Repeatative moving period (sec)

### Step 2) Run the follwing:
  ``` rosrun rviz_marker_publisher virtual_teleoperator.py```
  
