# Template: template-ros

This template provides a boilerplate repository
for developing ROS-based software in Duckietown.

**NOTE:** If you want to develop software that does not use
ROS, check out [this template](https://github.com/duckietown/template-basic).


## How to use it

### Create ROS Workspace and Package
- Create a ROS workspace

       $ mkdir -p ~/catkin_ws/src
       $ cd ~/catkin_ws/
       $ catkin_make
       $ source devel/setup.bash

- Run below commands to configure your ROS Package

       $ cd ~/catkin_ws
       $ catkin build
       $ source devel/setup.bash
    ### run the program
1. Object Detection and baal tracking
       
      - In shell 1:
     
       $ rosrun (how to run)
       
      - In shell 2:
     
       $ roscd Yadavbot/code/
       $ python publisher.py
       
      - In shell 3:
       
       $ dts start_gui_tools [Yadavbot]
       # rqt_image_view



***
