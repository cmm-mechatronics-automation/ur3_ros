# Install URSIM in ubuntu 2004

Download  URSIM from [here](https://www.universal-robots.com/download/software-cb-series/simulator-linux/offline-simulator-cb3-linux-ursim-3158/)


Problema para instalar, requiere java 8, daña instalación de ROS. se deberia instalar primero el simulador y luego si instalar ROS, esto para la version e, revisar si pasa lo mismo con la version que se tiene [link](https://forum.universal-robots.com/t/offline-simulator-e-series-ur-sim-for-linux-5-11-1-removes-all-installed-files/15384)


see [1](https://github.com/arunavanag591/ursim) step by step how to install.

## Install virtual Box 

see [info1](https://www.universal-robots.com/download/software-e-series/simulator-non-linux/offline-simulator-e-series-ur-sim-for-non-linux-594/), [info2](https://academy.universal-robots.com/media/r3xlna5e/ursim_vmoracle_installation_guide_v3_es.pdf) 

7z x URSim_VIRTUAL-3.14.3.1031232.rar


## Config ethernet in Vbox

Configurar una red Adaptador sólo-anfitrión (Host-only Adapter) y luego en la MV configurar este modo de operación de la red.

## How to use with the VM and ROS

ver [link](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md)

```
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.56.101

roslaunch ur_robot_driver example_rviz.launch
```



![](images/Ur3_ursim_ROS_noetic.gif)