# ur3_ros


## install


https://github.com/ros-industrial/universal_robot

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver


https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md


https://repositorio.cinvestav.mx/bitstream/handle/cinvestav/3881/SSIT0016938.pdf?sequence=1&isAllowed=y


## Usage

1. Configuring the robot and PC ip address
        Robot UR3: 192.168.56.2
        PC       : 192.168.56.2

2. Calibration file

```
    roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
```
  see the file in config/my_robot_calibration.yaml

3. launch the  driver

```
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.56.2
```

roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.56.2 \
  kinematics_config:=$(rospack find ur_calibration)/etc/ur10_example_calibration.yaml
