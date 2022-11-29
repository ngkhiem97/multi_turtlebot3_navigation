# multi_turtlebot3_navigation

Repository for controlling 2 Turtlebot3 robots in Vietnamese-German University's Robotics Lab

## Setup on Master Computer

```bash
cd ~/catkin_ws/src
git clone --branch master https://github.com/ngkhiem97/multi_turtlebot3_navigation.git
cd ~/catkin_ws
catkin_make
```

## First on Master Computer

```bash
roscore
```

## On SSH Robot clients' command lines

### Robot 1
```bash
ROS_NAMESPACE=turtle_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=turtle_1 set_frame_id:=turtle_1/base_scan
```

### Robot 2
```bash
ROS_NAMESPACE=turtle_2 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=turtle_2 set_frame_id:=turtle_2/base_scan
```

## On Master Computer

Launch navigation module (**amcl** and **move_base**) node on 2 robots. This will open Rviz interface
```bash
roslaunch multi_turtlebot3_navigation two_robots_navigation.launch
```

Open new terminal. Launch **navigator** for controlling robots based on shostest distance to goal
```bash
rosrun multi_turtlebot3_navigation navigator
```

Now you can controll the robots on Rviz interface using **2D pose estimate** for setting inital robots' location and **nav goal** to send navigation goal to the **navigator**. The navigator node will calculate the distance to the 2 robots and send the navigation goal to the robot that has the shortest distance
