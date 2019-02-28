# IIWA_IK_interface
This package provides a higher level as well as low level interface to IIWA. You can send the joint space positions or you can directly send the end-effector position and orientation.

"The read-me will be written!"
#  Dependencies
Eigen http://eigen.tuxfamily.org/index.php?title=Main_Page

Robot-toolkid https://github.com/epfl-lasa/robot-toolkit

Kinematic https://github.com/epfl-lasa/robot-kinematics
The rest of the required dependences are included in this package as git submodules.  

Note: Make sure that you have set-up the ssh option for github on your PC!

# Features:

- Joint space position level controller. 
- End-effector Pos/Orientation interface

# How to set-up the Robot

Note: You need to have set-up the [FRI_impedance](https://github.com/epfl-lasa/fri-iiwa-interface/blob/2dc70fee17f1740d4c5f7ab9a3f342e6c6d11ffe/FRI_Java_examples/FRI_Impdance.java) on the IIWA cabinet

The IP of the robot should be 192.170.10.2

The IP of the PC connected to the robot should be 192.170.10.1

The Net-mask of the PC connected to the robot should be 255.255.255.0

The Gateway of the PC connected to the robot should be 0.0.0.0.

# How to set-up the packge

## Installation
1. clone repository 
2. go inside IIWA_IK_interface folder
```
roscd 
cd ../src
git clone git@github.com:epfl-lasa/IIWA_IK_interface.git
roscd
cd ../src/IIWA_IK_interface
git submodule init
git submodule update
cd ../../
rospack profile
catkin_make
```
Note: If you are getting an error about a missing header, you might need to do ```catkin_make``` and ```rospack profile``` several times!

## Calibrate IIWA
Start calibration the PositionAndGMSRefrences scrip

# How to run

Run the robot:

In Termianl 1:
```
roscore
```
In Termianl 2:
```
rosrun fri_iiwa_ros fri_iiwa_example_ros
```
In Termianl 3; start robot simulator:
```
roscd
cd ../src/IIWA_IK_interface/robot-toolkit
./bin/robot_simulator --config packages/iiwa_scenarios/iiwa
```

# How to use:

#### robot_simulator interface
Possible robot_simulator commands:

##### job
Job position (robot with arms elveated). Go to before starting move.

##### init
Init inverse kinematic solver before going to move

##### move
The robot is ready to recieve position commands published to:
```
/IIWA/Desired_E_Pos
```
You can read the end-position by subscribing to:
```
/IIWA/Real_E_Pos
```

## Copyright
More information regarding the IK solver is available in this paper. 
For more information contact [Sina Mirrazavi](http://lasa.epfl.ch/people/member.php?SCIPER=233855).

