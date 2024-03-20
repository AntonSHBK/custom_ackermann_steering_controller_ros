# Custom ackerman steering controller for mobile robot 4 wheel ros Noetic

<img src="" alt="Привью" height="400">
<img src="" alt="GIF_Move" height="400">

- [Russian description](RU.md)

Create container:
```bush
cd  docker
docker-compose build
docker-compose up
```

Build project:
```
catkin_make
```

Run project:
```bush
source devel/setup.bash
roslaunch custom_ackermann_steering_controller_ros ackermann_run.launch
```