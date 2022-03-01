# rumba_autocar
navigation practice using roomba

## dependence package
[AutonamyLab/create_robot](https://github.com/AutonomyLab/create_robot)  
[tamago117/control_panel_plugin](https://github.com/tamago117/control_panel_plugin)
[tamago117/eg_navigation](https://github.com/tamago117/eg_navigation)

## Installation
```
cd catkin_ws/src
git clone https://github.com/tamago117/rumba_autocar.git
cd ..
catkin_make
```

## simple demo
```
roslaunch rumba_autocar navigation_2d.launch

#real roomba
roslaunch rumba_autocar navigation_2d.launch sim:=true
```
https://user-images.githubusercontent.com/38370926/134785348-0740eb23-357f-4d7b-a2ae-2db73768416a.mp4

