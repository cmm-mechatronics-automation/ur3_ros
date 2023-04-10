# Attaching an end effector to the robot


## To spawn the robot in Rviz


```
roslaunch ur_description ur3_upload.launch
roslaunch ur_description view_ur3.launch 
```

## To spawn the robot in Gazebo


```
roslaunch ur_gazebo ur3_bringup.launch
```


## To control the robot with `ros_control`



### Gazebo 

The package `ur_gazebo` includes a file, `load_ur.launch.xml`, that pretends to load a driver for a UR robot, starting the driver node, in this case the Gazebo model, and loading the `ros_control` controllers. Indeed, the `load_ur3.launch.xml` call it with the appropriate arguments given in the file `ur3_bringup.launch`.















---

# References

[UR3_with_Robotiq_gripper](https://github.com/trabelsim/UR3_with_Robotiq_gripper_85)

[roboticscasual](https://roboticscasual.com/ros-tutorial-visualize-ur5-in-rviz-urdf-explained/)