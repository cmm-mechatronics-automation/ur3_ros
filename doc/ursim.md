# Install URSIM in ubuntu 2004

Download  URSIM from [here](https://www.universal-robots.com/download/software-cb-series/simulator-linux/offline-simulator-cb3-linux-ursim-3158/)

Installing URSim in ubuntu, according to [here](https://forum.universal-robots.com/t/offline-simulator-e-series-ur-sim-for-linux-5-11-1-removes-all-installed-files/15384), breaks the ROS installation. Then, install the simulator URSim first, and then install ROS, at least for for the e-version.  It should be checked this condition happens with the CB version.

see [1](https://github.com/arunavanag591/ursim), [2](https://www.mathworks.com/help/supportpkg/urseries/ug/setup-ursim-offline-simulator.html) step by step how to install URSim.

Instead we install VM.

# Install virtual Box 

see [info VE](https://www.universal-robots.com/download/software-e-series/simulator-non-linux/offline-simulator-e-series-ur-sim-for-non-linux-594/),[info V CB](https://www.universal-robots.com/download/software-cb-series/simulator-non-linux/offline-simulator-cb-series-non-linux-ursim-3158/) ,[info2 doc](https://academy.universal-robots.com/media/r3xlna5e/ursim_vmoracle_installation_guide_v3_es.pdf) 

```
7z x URSim_VIRTUAL-3.14.3.1031232.rar
```

## Config ethernet in Vbox

Configurar una red __Adaptador sólo-anfitrión (Host-only Adapter)__ y luego en la MV configurar este modo de operación de la red.

## How to use with the VM and ROS

ver [link](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md)

Run the VM  and open URSim.

```
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.56.102

roslaunch ur_robot_driver example_rviz.launch
```


![](images/Ur3_ursim_ROS_noetic.gif)



## Use moveit! 
```
roslaunch ur3_moveit_config moveit_planning_execution.launch sim:=true

roslaunch ur3_moveit_config moveit_rviz.launch
```

Luego de hacer la planeación en rviz, sale el error


```
...

[ INFO] [1678904220.911134129]: Execution request received
[ WARN] [1678904220.919861759]: Controller 'scaled_pos_joint_traj_controller' failed with error INVALID_GOAL: 
[ WARN] [1678904220.919935262]: Controller handle scaled_pos_joint_traj_controller reports status FAILED
[ INFO] [1678904220.919964724]: Completed trajectory execution with status FAILED ...
[ INFO] [1678904220.920016774]: Execution completed: FAILED

...

```

[ver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/55) En el paquete universal_robot/ur*_moveit_config/config/controllers.yaml, cambiar la linea 

```
action_ns: follow_joint_trajectory
```

a


```
action_ns: follow_joint_trajectory

```



## URSIM for UrCaps development

see [here](https://www.universal-robots.com/articles/ur/urplus-resources/urcap-how-to-install-ursim-for-urcaps-development/)



## URSIM and Matlab

see Matlab [doc1](https://www.mathworks.com/help/supportpkg/urseries/ug/setup-ursim-offline-simulator.html)[doc2](https://github.com/mathworks/MATLAB-URCap-for-External-Control)
