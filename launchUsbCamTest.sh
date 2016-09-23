#!/bin/bash

# TODO add custom catkin_ws path, currently assuming it's in ~


#--disable-factory needed for getting pid of background terminal

gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscore" ' &

sleep 4

gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscd usb_cam && cd launch/ &&  roslaunch usb_cam-testWithVisu.launch " ' &


trap 'kill $(jobs -pr) && clear' SIGINT SIGTERM EXIT

