# Custom ackerman steering controller for mobile robot 4 wheel ros Noetic

In this paper, we consider an example of creating simulations of a model of a four-wheeled mobile platform with steering according to the type of Accremann, using the `ROS` framework. The `Gazebo` is selected as the simulation environment. Examples of the description of the robot architecture are given, using the `UDF` format, a proprietary controller has been developed to control the wheels of the model, and a general approach to the design of simulation models of robotic systems is shown.

## [Russian description](RU.md)

## [Видео YouTube](https://www.youtube.com/watch?v=F9TGboiuFvM&t=59s)

<img src="imgs/rviz_1.jpg" alt="Привью" height="400">
<img src="imgs/rviz_gazebo_1.gif" alt="GIF_Move" height="400">


## Create container:
```bush
cd  docker
docker-compose build
docker-compose up
```

## Build project:
```
catkin_make
```

## Run project:
```bush
source devel/setup.bash
roslaunch custom_ackermann_steering_controller_ros ackermann_run.launch
```
Run in another terminal (keyboard control)
```bush
source devel/setup.bash
rosrun custom_ackermann_steering_controller_ros keyboard_teleop.py
```