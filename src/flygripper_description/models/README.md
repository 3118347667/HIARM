# Models
grasped object are compatible with the urdf and sdf file:
- coke_can
- mustard_bottle
- soccer_ball
- unit_box (basic shape for testing)

rest are stationary objects:
- table
- lamp_post

# Tips to transform sdf to urdf
We can use pysdf to convert sdf to urdf by running:
```bash
rosrun pysdf sdf2urdf.py <sdf_file> <urdf_file>
```

modify parse.py in pysdf to add the following lines to the end of the file:
```python
catkin_ws_path = os.path.expanduser('~') + '/flygripper_robotics/src/'
```

where '/flygripper_robotics/src/' is the path to the src folder of the catkin workspace