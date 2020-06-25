# rviz_marker_publisher

This is a simple ROS publisher for MarkerArray in Rviz. The current version can publish multiple spherical markers. 


## (1) Obstacles Generation

### Step 1) Modify "Marker Array Setting" in `gen_obstacles.py`

- `radius`: the radius of each sphere
- `num_obstacles`: the number of spheres that you are going to visualise
- `range_obstacles`: the Cartesian range where the spheres will be uniform-randomly appeared. It should be such that  `(x_min, x_max, y_min, y_max, z_min, z_max)`

### Step 2) Mode selection

- `mode`: 
  - `Manual` mode allows to modify the obstacles' positions by pressing `w`, `x`, `a`, `d`, `q`, `z`.
  - `RandomMove` makes the obstacles slightly move randomly. 

### Step 3) Run the following:

  ``` rosrun rviz_marker_publisher gen_obstacles.py```

The result MarkerArray will be published via `visualization_marker_array` topic. 


## (2) Virtual teleoprator

### Step 1) Select the control mode (`mode`)  in `virtual_teleoprator.py`
- `mode == 1`: "8"-shaped periodic moving. (Go to Step 2-1)
- `mode == 2`: Random movement (Go to Step 2-2)

### Step 2-1) Periodic movement setting for Mode 1

- `xyzrpy`: (m/s) XYZ velocity // (rad) Amplitudes of Roll Pitch Yaw in End Effector Frame
- `moving_period`: Repeatative moving period (sec)

### Step 2-2) Random movement setting for Mode 2
- `right_range`: the range where each random target pose will be generated
- `max_del_x`: a sense of velocity of the movement
- `moving_period_mode_two`: Repeatative moving period (sec) 

### Step 2) Run the follwing:
  ``` rosrun rviz_marker_publisher virtual_teleoperator.py```
  
## Example
See the video (https://youtu.be/Ki-miu0sTp4), which uses `RandomMove` for `gen_obstacles.py` and `mode = 2` for `virtual_teleoprator.py`
