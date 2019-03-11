# Mechatronics_project

Repository including the ROS packages of our project.

Git clone the gazebo_sim package to keep up to date with the changes is out program.


# Complete procedure to run the simulation in Ubuntu 16.40.3 LTS

1. ROS Kinetic installation.
http://wiki.ros.org/kinetic/Installation/Ubuntu 

2. Install package hector_gazebo
```
sudo apt-get install ros-kinetic-hector-gazebo
```
3. Install package hector_localization
```
sudo apt-get install ros-kinetic-hector-localization
```

4. Install package hector_slam
```
sudo apt-get install ros-kinetic-hector-slam
```

5. Install package hector_models
```
sudo apt-get install ros-kinetic-hector-models
```

6. Clone the hector_quadrotor package in your workspace
```
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
```

  6.1. Install package hardware_interface

  6.2. Install package Controller_interface

  6.3. Install package gazebo_ros_control

7. Install package ar_alvar_track

8. Clone the bebop_simulator package
```
git clone git clone https://github.com/gstavrinos/tf_velocity_estimator.git
```

9. Clone the ar_helipad package
```
git clone https://github.com/gstavrinos/aerial_local_planner.git
```

10. Clone the tf_velocity_estimator package
```
git clone https://github.com/gstavrinos/aerial_global_planner.git
```

11. Install package twist_mux package
```
sudo apt-get install ros-kinetic-twist-mux
```

12. 11. Install package ros_control
```
sudo apt-get install ros-kinetic-ros-control
```

13. In case of need, install untangle python library and pip
```
sudo apt install python-pip
pip install --upgrade pip
pip install untangle
```



Notes:
You may also install the joy package.

How to install deb packages ubuntu    
```
sudo dpkg -i DEB_PACKAGE
```
