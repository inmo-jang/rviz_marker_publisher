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
- `mode == 2`: Random movement (Go to Step 2-2) -- NOTE: This includes the function of `gen_obstacles.py`, thus it is self-contained, i.e. you do not need to run `gen_obstacles.py` separately. This mode is for automatically collecting data for the paper. 

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


## (3) Encapsulatting gazebo objects (`encapsulation.py`)

### Step 1) Define object keywords and capsule sizes to encapsulate them 
  - `keywords_of_object_to_encapsulate`: a list of the keyword strings (e.g. If you put `"object"` and any gazebo model object whose name includes this keyword will be encapsulated)
  - `capsules_sizes`: a list of the capsule sizes for each keyword. NOTE: Currently, only ellipsoid capsule is available thus the capsule size is supposed to be its diameter for each axis. 
  - `robot_height`: the z-axis offset between the IK solver's coordination frame and the Gazebo environment's frame. 
  
### Step 2) Run the following:
  ```
  rosrun rviz_marker_publisher encapsulation.py
  ```
