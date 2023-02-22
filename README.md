# ur3_ros

## Resources:


https://github.com/ros-industrial/universal_robot

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver


https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md

```
Polyscope: 3.15.8.106339 

UrCaps - external control - V1.0.5
```

## install


https://github.com/ros-industrial/universal_robot

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver


https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md


https://repositorio.cinvestav.mx/bitstream/handle/cinvestav/3881/SSIT0016938.pdf?sequence=1&isAllowed=y


## Usage
Please see this for an example to use ur3 with ROS [link](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md)

1. Configuring the robot and PC ip address

```
        Mascara subred: 255.255.255.0

        Puerta de enlace predeterminada: 192.168.56.1

        Robot UR3: 192.168.56.2

        PC       : 192.168.56.101:50002
```

2. Calibration file

```
    roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=192.168.56.2 target_filename:="${HOME}/my_robot_calibration.yaml"
```

roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.56.2 target_filename:="${HOME}/my_robot_calibration.yaml"

  see the file in config/my_robot_calibration.yaml

3. launch the  driver

```
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.56.2 kinematics_config:="${HOME}/my_robot_calibration.yaml"
```


error:

```
INFO] [1671486277.395867]: Controller Spawner: Waiting for service controller_manager/load_controller
[FATAL] [1671486283.401838934]: Variable 'speed_slider_mask' is currently controlled by another RTDE client. The input recipe can't be used as configured
[ERROR] [1671486283.401968453]: Could not correctly initialize robot. Exiting
================================================================================REQUIRED process [ur_hardware_interface-3] has died!
process has died [pid 14056, exit code 1, cmd /home/ros1/catkin_ws/devel/lib/ur_robot_driver/ur_robot_driver_node __name:=ur_hardware_interface __log:=/home/ros1/.ros/log/547d2a5e-7fe6-11ed-bfc9-0178fc8172fe/ur_hardware_interface-3.log].
log file: /home/ros1/.ros/log/547d2a5e-7fe6-11ed-bfc9-0178fc8172fe/ur_hardware_interface-3*.log
Initiating shutdown!
================================================================================
[ur_hardware_interface/ur_robot_state_helper-7] killing on exit
[controller_stopper-6] killing on exit
```

```


  $ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.56.2 \
>   kinematics_config:="${HOME}/my_robot_calibration.yaml"
... logging to /home/ros1/.ros/log/d026b368-a7d3-11ed-b7f1-2993f8114d47/roslaunch-ros1-HP-Z4-G4-Workstation-6841.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ros1-HP-Z4-G4-Workstation:35083/

SUMMARY
========

PARAMETERS
 * /controller_stopper/consistent_controllers: ['joint_state_con...
 * /force_torque_sensor_controller/publish_rate: 125
 * /force_torque_sensor_controller/type: force_torque_sens...
 * /forward_cartesian_traj_controller/joints: ['shoulder_pan_jo...
 * /forward_cartesian_traj_controller/type: pass_through_cont...
 * /forward_joint_traj_controller/joints: ['shoulder_pan_jo...
 * /forward_joint_traj_controller/type: pass_through_cont...
 * /hardware_control_loop/loop_hz: 125
 * /joint_based_cartesian_traj_controller/base: base
 * /joint_based_cartesian_traj_controller/joints: ['shoulder_pan_jo...
 * /joint_based_cartesian_traj_controller/tip: tool0
 * /joint_based_cartesian_traj_controller/type: position_controll...
 * /joint_group_vel_controller/joints: ['shoulder_pan_jo...
 * /joint_group_vel_controller/type: velocity_controll...
 * /joint_state_controller/publish_rate: 125
 * /joint_state_controller/type: joint_state_contr...
 * /pos_joint_traj_controller/action_monitor_rate: 20
 * /pos_joint_traj_controller/constraints/elbow_joint/goal: 0.1
 * /pos_joint_traj_controller/constraints/elbow_joint/trajectory: 0.2
 * /pos_joint_traj_controller/constraints/goal_time: 0.6
 * /pos_joint_traj_controller/constraints/shoulder_lift_joint/goal: 0.1
 * /pos_joint_traj_controller/constraints/shoulder_lift_joint/trajectory: 0.2
 * /pos_joint_traj_controller/constraints/shoulder_pan_joint/goal: 0.1
 * /pos_joint_traj_controller/constraints/shoulder_pan_joint/trajectory: 0.2
 * /pos_joint_traj_controller/constraints/stopped_velocity_tolerance: 0.05
 * /pos_joint_traj_controller/constraints/wrist_1_joint/goal: 0.1
 * /pos_joint_traj_controller/constraints/wrist_1_joint/trajectory: 0.2
 * /pos_joint_traj_controller/constraints/wrist_2_joint/goal: 0.1
 * /pos_joint_traj_controller/constraints/wrist_2_joint/trajectory: 0.2
 * /pos_joint_traj_controller/constraints/wrist_3_joint/goal: 0.1
 * /pos_joint_traj_controller/constraints/wrist_3_joint/trajectory: 0.2
 * /pos_joint_traj_controller/joints: ['shoulder_pan_jo...
 * /pos_joint_traj_controller/state_publish_rate: 125
 * /pos_joint_traj_controller/stop_trajectory_duration: 0.5
 * /pos_joint_traj_controller/type: position_controll...
 * /pose_based_cartesian_traj_controller/base: base
 * /pose_based_cartesian_traj_controller/joints: ['shoulder_pan_jo...
 * /pose_based_cartesian_traj_controller/tip: tool0_controller
 * /pose_based_cartesian_traj_controller/type: pose_controllers/...
 * /robot_description: <?xml version="1....
 * /robot_status_controller/handle_name: industrial_robot_...
 * /robot_status_controller/publish_rate: 10
 * /robot_status_controller/type: industrial_robot_...
 * /rosdistro: noetic
 * /rosversion: 1.15.15
 * /scaled_pos_joint_traj_controller/action_monitor_rate: 20
 * /scaled_pos_joint_traj_controller/constraints/elbow_joint/goal: 0.1
 * /scaled_pos_joint_traj_controller/constraints/elbow_joint/trajectory: 0.2
 * /scaled_pos_joint_traj_controller/constraints/goal_time: 0.6
 * /scaled_pos_joint_traj_controller/constraints/shoulder_lift_joint/goal: 0.1
 * /scaled_pos_joint_traj_controller/constraints/shoulder_lift_joint/trajectory: 0.2
 * /scaled_pos_joint_traj_controller/constraints/shoulder_pan_joint/goal: 0.1
 * /scaled_pos_joint_traj_controller/constraints/shoulder_pan_joint/trajectory: 0.2
 * /scaled_pos_joint_traj_controller/constraints/stopped_velocity_tolerance: 0.05
 * /scaled_pos_joint_traj_controller/constraints/wrist_1_joint/goal: 0.1
 * /scaled_pos_joint_traj_controller/constraints/wrist_1_joint/trajectory: 0.2
 * /scaled_pos_joint_traj_controller/constraints/wrist_2_joint/goal: 0.1
 * /scaled_pos_joint_traj_controller/constraints/wrist_2_joint/trajectory: 0.2
 * /scaled_pos_joint_traj_controller/constraints/wrist_3_joint/goal: 0.1
 * /scaled_pos_joint_traj_controller/constraints/wrist_3_joint/trajectory: 0.2
 * /scaled_pos_joint_traj_controller/joints: ['shoulder_pan_jo...
 * /scaled_pos_joint_traj_controller/state_publish_rate: 125
 * /scaled_pos_joint_traj_controller/stop_trajectory_duration: 0.5
 * /scaled_pos_joint_traj_controller/type: position_controll...
 * /scaled_vel_joint_traj_controller/action_monitor_rate: 20
 * /scaled_vel_joint_traj_controller/constraints/elbow_joint/goal: 0.1
 * /scaled_vel_joint_traj_controller/constraints/elbow_joint/trajectory: 0.1
 * /scaled_vel_joint_traj_controller/constraints/goal_time: 0.6
 * /scaled_vel_joint_traj_controller/constraints/shoulder_lift_joint/goal: 0.1
 * /scaled_vel_joint_traj_controller/constraints/shoulder_lift_joint/trajectory: 0.1
 * /scaled_vel_joint_traj_controller/constraints/shoulder_pan_joint/goal: 0.1
 * /scaled_vel_joint_traj_controller/constraints/shoulder_pan_joint/trajectory: 0.1
 * /scaled_vel_joint_traj_controller/constraints/stopped_velocity_tolerance: 0.05
 * /scaled_vel_joint_traj_controller/constraints/wrist_1_joint/goal: 0.1
 * /scaled_vel_joint_traj_controller/constraints/wrist_1_joint/trajectory: 0.1
 * /scaled_vel_joint_traj_controller/constraints/wrist_2_joint/goal: 0.1
 * /scaled_vel_joint_traj_controller/constraints/wrist_2_joint/trajectory: 0.1
 * /scaled_vel_joint_traj_controller/constraints/wrist_3_joint/goal: 0.1
 * /scaled_vel_joint_traj_controller/constraints/wrist_3_joint/trajectory: 0.1
 * /scaled_vel_joint_traj_controller/gains/elbow_joint/d: 0.1
 * /scaled_vel_joint_traj_controller/gains/elbow_joint/i: 0.05
 * /scaled_vel_joint_traj_controller/gains/elbow_joint/i_clamp: 1
 * /scaled_vel_joint_traj_controller/gains/elbow_joint/p: 5.0
 * /scaled_vel_joint_traj_controller/gains/shoulder_lift_joint/d: 0.1
 * /scaled_vel_joint_traj_controller/gains/shoulder_lift_joint/i: 0.05
 * /scaled_vel_joint_traj_controller/gains/shoulder_lift_joint/i_clamp: 1
 * /scaled_vel_joint_traj_controller/gains/shoulder_lift_joint/p: 5.0
 * /scaled_vel_joint_traj_controller/gains/shoulder_pan_joint/d: 0.1
 * /scaled_vel_joint_traj_controller/gains/shoulder_pan_joint/i: 0.05
 * /scaled_vel_joint_traj_controller/gains/shoulder_pan_joint/i_clamp: 1
 * /scaled_vel_joint_traj_controller/gains/shoulder_pan_joint/p: 5.0
 * /scaled_vel_joint_traj_controller/gains/wrist_1_joint/d: 0.1
 * /scaled_vel_joint_traj_controller/gains/wrist_1_joint/i: 0.05
 * /scaled_vel_joint_traj_controller/gains/wrist_1_joint/i_clamp: 1
 * /scaled_vel_joint_traj_controller/gains/wrist_1_joint/p: 5.0
 * /scaled_vel_joint_traj_controller/gains/wrist_2_joint/d: 0.1
 * /scaled_vel_joint_traj_controller/gains/wrist_2_joint/i: 0.05
 * /scaled_vel_joint_traj_controller/gains/wrist_2_joint/i_clamp: 1
 * /scaled_vel_joint_traj_controller/gains/wrist_2_joint/p: 5.0
 * /scaled_vel_joint_traj_controller/gains/wrist_3_joint/d: 0.1
 * /scaled_vel_joint_traj_controller/gains/wrist_3_joint/i: 0.05
 * /scaled_vel_joint_traj_controller/gains/wrist_3_joint/i_clamp: 1
 * /scaled_vel_joint_traj_controller/gains/wrist_3_joint/p: 5.0
 * /scaled_vel_joint_traj_controller/joints: ['shoulder_pan_jo...
 * /scaled_vel_joint_traj_controller/state_publish_rate: 125
 * /scaled_vel_joint_traj_controller/stop_trajectory_duration: 0.5
 * /scaled_vel_joint_traj_controller/type: velocity_controll...
 * /scaled_vel_joint_traj_controller/velocity_ff/elbow_joint: 1.0
 * /scaled_vel_joint_traj_controller/velocity_ff/shoulder_lift_joint: 1.0
 * /scaled_vel_joint_traj_controller/velocity_ff/shoulder_pan_joint: 1.0
 * /scaled_vel_joint_traj_controller/velocity_ff/wrist_1_joint: 1.0
 * /scaled_vel_joint_traj_controller/velocity_ff/wrist_2_joint: 1.0
 * /scaled_vel_joint_traj_controller/velocity_ff/wrist_3_joint: 1.0
 * /speed_scaling_state_controller/publish_rate: 125
 * /speed_scaling_state_controller/type: scaled_controller...
 * /twist_controller/frame_id: tool0_controller
 * /twist_controller/joints: ['shoulder_pan_jo...
 * /twist_controller/publish_rate: 125
 * /twist_controller/type: ros_controllers_c...
 * /ur_hardware_interface/headless_mode: False
 * /ur_hardware_interface/input_recipe_file: /home/ros1/catkin...
 * /ur_hardware_interface/joints: ['shoulder_pan_jo...
 * /ur_hardware_interface/kinematics/forearm/pitch: -3.1397251889124
 * /ur_hardware_interface/kinematics/forearm/roll: 3.140880702024851
 * /ur_hardware_interface/kinematics/forearm/x: -0.2434794139874107
 * /ur_hardware_interface/kinematics/forearm/y: 0
 * /ur_hardware_interface/kinematics/forearm/yaw: -3.141567687120497
 * /ur_hardware_interface/kinematics/forearm/z: 0
 * /ur_hardware_interface/kinematics/hash: calib_33618816080...
 * /ur_hardware_interface/kinematics/shoulder/pitch: 0
 * /ur_hardware_interface/kinematics/shoulder/roll: 0
 * /ur_hardware_interface/kinematics/shoulder/x: 0
 * /ur_hardware_interface/kinematics/shoulder/y: 0
 * /ur_hardware_interface/kinematics/shoulder/yaw: -4.07525611140682...
 * /ur_hardware_interface/kinematics/shoulder/z: 0.1519720094709559
 * /ur_hardware_interface/kinematics/upper_arm/pitch: 0
 * /ur_hardware_interface/kinematics/upper_arm/roll: 1.571184458807471
 * /ur_hardware_interface/kinematics/upper_arm/x: -0.00017417242210...
 * /ur_hardware_interface/kinematics/upper_arm/y: 0
 * /ur_hardware_interface/kinematics/upper_arm/yaw: -3.78063086473393...
 * /ur_hardware_interface/kinematics/upper_arm/z: 0
 * /ur_hardware_interface/kinematics/wrist_1/pitch: 0.000497233162916...
 * /ur_hardware_interface/kinematics/wrist_1/roll: 0.004257494186500489
 * /ur_hardware_interface/kinematics/wrist_1/x: -0.2132902672678847
 * /ur_hardware_interface/kinematics/wrist_1/y: -0.00047695101764...
 * /ur_hardware_interface/kinematics/wrist_1/yaw: -0.00013128636383...
 * /ur_hardware_interface/kinematics/wrist_1/z: 0.1120255518791114
 * /ur_hardware_interface/kinematics/wrist_2/pitch: 0
 * /ur_hardware_interface/kinematics/wrist_2/roll: 1.571676254802948
 * /ur_hardware_interface/kinematics/wrist_2/x: 6.822933753254518...
 * /ur_hardware_interface/kinematics/wrist_2/y: -0.08540737301934301
 * /ur_hardware_interface/kinematics/wrist_2/yaw: 6.512858624217049...
 * /ur_hardware_interface/kinematics/wrist_2/z: -7.51523590099423...
 * /ur_hardware_interface/kinematics/wrist_3/pitch: 3.141592653589793
 * /ur_hardware_interface/kinematics/wrist_3/roll: 1.57023238419285
 * /ur_hardware_interface/kinematics/wrist_3/x: -3.70839658795363...
 * /ur_hardware_interface/kinematics/wrist_3/y: 0.08227790421633105
 * /ur_hardware_interface/kinematics/wrist_3/yaw: -3.141566807022587
 * /ur_hardware_interface/kinematics/wrist_3/z: -4.64000203135717...
 * /ur_hardware_interface/output_recipe_file: /home/ros1/catkin...
 * /ur_hardware_interface/reverse_ip: 
 * /ur_hardware_interface/reverse_port: 50001
 * /ur_hardware_interface/robot_ip: 192.168.56.2
 * /ur_hardware_interface/script_command_port: 50004
 * /ur_hardware_interface/script_file: /opt/ros/noetic/s...
 * /ur_hardware_interface/script_sender_port: 50002
 * /ur_hardware_interface/servoj_gain: 2000
 * /ur_hardware_interface/servoj_lookahead_time: 0.03
 * /ur_hardware_interface/tf_prefix: 
 * /ur_hardware_interface/tool_baud_rate: 115200
 * /ur_hardware_interface/tool_parity: 0
 * /ur_hardware_interface/tool_rx_idle_chars: 1.5
 * /ur_hardware_interface/tool_stop_bits: 1
 * /ur_hardware_interface/tool_tx_idle_chars: 3.5
 * /ur_hardware_interface/tool_voltage: 0
 * /ur_hardware_interface/trajectory_port: 50003
 * /ur_hardware_interface/use_tool_communication: False
 * /vel_joint_traj_controller/action_monitor_rate: 20
 * /vel_joint_traj_controller/constraints/elbow_joint/goal: 0.1
 * /vel_joint_traj_controller/constraints/elbow_joint/trajectory: 0.1
 * /vel_joint_traj_controller/constraints/goal_time: 0.6
 * /vel_joint_traj_controller/constraints/shoulder_lift_joint/goal: 0.1
 * /vel_joint_traj_controller/constraints/shoulder_lift_joint/trajectory: 0.1
 * /vel_joint_traj_controller/constraints/shoulder_pan_joint/goal: 0.1
 * /vel_joint_traj_controller/constraints/shoulder_pan_joint/trajectory: 0.1
 * /vel_joint_traj_controller/constraints/stopped_velocity_tolerance: 0.05
 * /vel_joint_traj_controller/constraints/wrist_1_joint/goal: 0.1
 * /vel_joint_traj_controller/constraints/wrist_1_joint/trajectory: 0.1
 * /vel_joint_traj_controller/constraints/wrist_2_joint/goal: 0.1
 * /vel_joint_traj_controller/constraints/wrist_2_joint/trajectory: 0.1
 * /vel_joint_traj_controller/constraints/wrist_3_joint/goal: 0.1
 * /vel_joint_traj_controller/constraints/wrist_3_joint/trajectory: 0.1
 * /vel_joint_traj_controller/gains/elbow_joint/d: 0.1
 * /vel_joint_traj_controller/gains/elbow_joint/i: 0.05
 * /vel_joint_traj_controller/gains/elbow_joint/i_clamp: 1
 * /vel_joint_traj_controller/gains/elbow_joint/p: 5.0
 * /vel_joint_traj_controller/gains/shoulder_lift_joint/d: 0.1
 * /vel_joint_traj_controller/gains/shoulder_lift_joint/i: 0.05
 * /vel_joint_traj_controller/gains/shoulder_lift_joint/i_clamp: 1
 * /vel_joint_traj_controller/gains/shoulder_lift_joint/p: 5.0
 * /vel_joint_traj_controller/gains/shoulder_pan_joint/d: 0.1
 * /vel_joint_traj_controller/gains/shoulder_pan_joint/i: 0.05
 * /vel_joint_traj_controller/gains/shoulder_pan_joint/i_clamp: 1
 * /vel_joint_traj_controller/gains/shoulder_pan_joint/p: 5.0
 * /vel_joint_traj_controller/gains/wrist_1_joint/d: 0.1
 * /vel_joint_traj_controller/gains/wrist_1_joint/i: 0.05
 * /vel_joint_traj_controller/gains/wrist_1_joint/i_clamp: 1
 * /vel_joint_traj_controller/gains/wrist_1_joint/p: 5.0
 * /vel_joint_traj_controller/gains/wrist_2_joint/d: 0.1
 * /vel_joint_traj_controller/gains/wrist_2_joint/i: 0.05
 * /vel_joint_traj_controller/gains/wrist_2_joint/i_clamp: 1
 * /vel_joint_traj_controller/gains/wrist_2_joint/p: 5.0
 * /vel_joint_traj_controller/gains/wrist_3_joint/d: 0.1
 * /vel_joint_traj_controller/gains/wrist_3_joint/i: 0.05
 * /vel_joint_traj_controller/gains/wrist_3_joint/i_clamp: 1
 * /vel_joint_traj_controller/gains/wrist_3_joint/p: 5.0
 * /vel_joint_traj_controller/joints: ['shoulder_pan_jo...
 * /vel_joint_traj_controller/state_publish_rate: 125
 * /vel_joint_traj_controller/stop_trajectory_duration: 0.5
 * /vel_joint_traj_controller/type: velocity_controll...
 * /vel_joint_traj_controller/velocity_ff/elbow_joint: 1.0
 * /vel_joint_traj_controller/velocity_ff/shoulder_lift_joint: 1.0
 * /vel_joint_traj_controller/velocity_ff/shoulder_pan_joint: 1.0
 * /vel_joint_traj_controller/velocity_ff/wrist_1_joint: 1.0
 * /vel_joint_traj_controller/velocity_ff/wrist_2_joint: 1.0
 * /vel_joint_traj_controller/velocity_ff/wrist_3_joint: 1.0

NODES
  /
    controller_stopper (controller_stopper/node)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    ros_control_controller_spawner (controller_manager/spawner)
    ros_control_stopped_spawner (controller_manager/spawner)
    ur_hardware_interface (ur_robot_driver/ur_robot_driver_node)
  /ur_hardware_interface/
    ur_robot_state_helper (ur_robot_driver/robot_state_helper)

auto-starting new master
process[master]: started with pid [6851]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to d026b368-a7d3-11ed-b7f1-2993f8114d47
process[rosout-1]: started with pid [6861]
started core service [/rosout]
process[robot_state_publisher-2]: started with pid [6868]
process[ur_hardware_interface-3]: started with pid [6869]
process[ros_control_controller_spawner-4]: started with pid [6870]
process[ros_control_stopped_spawner-5]: started with pid [6871]
process[controller_stopper-6]: started with pid [6874]
[ INFO] [1675876370.561772589]: Waiting for controller manager service to come up on /controller_manager/switch_controller
[ INFO] [1675876370.563714261]: waitForService: Service [/controller_manager/switch_controller] has not been advertised, waiting...
process[ur_hardware_interface/ur_robot_state_helper-7]: started with pid [6875]
[ INFO] [1675876370.849359290]: Initializing dashboard client
[ INFO] [1675876370.853077292]: Connected: Universal Robots Dashboard Server

[ INFO] [1675876370.855075358]: waitForService: Service [/ur_hardware_interface/dashboard/play] has not been advertised, waiting...
[ INFO] [1675876370.879036742]: waitForService: Service [/ur_hardware_interface/dashboard/play] is now available.
[ INFO] [1675876370.896384267]: Initializing urdriver
[ WARN] [1675876370.921678839]: No realtime capabilities found. Consider using a realtime system for better performance
[INFO] [1675876371.082567]: Controller Spawner: Waiting for service controller_manager/load_controller
[INFO] [1675876371.082840]: Controller Spawner: Waiting for service controller_manager/load_controller
[ INFO] [1675876371.363437675]: Negotiated RTDE protocol version to 2.
[ INFO] [1675876371.363771774]: Setting up RTDE communication with frequency 125.000000
================================================================================REQUIRED process [ur_hardware_interface-3] has died!
process has died [pid 6869, exit code -11, cmd /home/ros1/catkin_ws/devel/lib/ur_robot_driver/ur_robot_driver_node __name:=ur_hardware_interface __log:=/home/ros1/.ros/log/d026b368-a7d3-11ed-b7f1-2993f8114d47/ur_hardware_interface-3.log].
log file: /home/ros1/.ros/log/d026b368-a7d3-11ed-b7f1-2993f8114d47/ur_hardware_interface-3*.log
Initiating shutdown!
================================================================================
[ur_hardware_interface/ur_robot_state_helper-7] killing on exit
[controller_stopper-6] killing on exit
[ros_control_stopped_spawner-5] killing on exit
[ros_control_controller_spawner-4] killing on exit
[ur_hardware_interface-3] killing on exit
[robot_state_publisher-2] killing on exit
[WARN] [1675876372.345886]: Controller Spawner couldn't find the expected controller_manager ROS interface.
[WARN] [1675876372.346760]: Controller Spawner couldn't find the expected controller_manager ROS interface.
^C^C[controller_stopper-6] escalating to SIGTERM
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

1.  Se revisa la intalacion del Control Externo URCap [aqui](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)

<!-- 1. De acuerdo con el [link](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md) se necesita minimo polyscope 3.7. Se debe actualizar este! ver [link](https://www.universal-robots.com/articles/ur/how-to-videos/polyscope-software-update-on-cb3/) -->


sudo ufw status: inactive ##firewall disable

para revisar

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/590
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/589
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/509
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/441
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/440
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/272
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/254
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/182


## Config UrCaps

see [link](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)

## HElp

se [UR forum](https://forum.universal-robots.com/c/ros/23) 