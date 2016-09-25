#!/bin/bash

# TODO add custom catkin_ws path, currently assuming it's in ~

# # change these paths if needed!
# ORB_SLAM_ROOT=/home/epflcvlab/programs/ORB_SLAM2

# #--disable-factory needed for getting pid of background terminal
# gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscore" ' &
# # pid0=$!

# sleep 4

# gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscd usb_cam && cd launch/ && roslaunch usb_cam-test.launch " ' &

# COMMAND='/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${ORB_SLAM_INSTALL_ROOT}/Examples/ROS && cd ${ORB_SLAM_INSTALL_ROOT} && rosrun SMART_ORB_SLAM2 Mono Vocabulary/ORBvoc.txt ./Examples/ROS/SMART_ORB_SLAM2/SMART_ORBSLAM2_Logitec9000.yaml" '
# ORB_SLAM_INSTALL_ROOT=${ORB_SLAM_ROOT}  gnome-terminal --disable-factory -e "${COMMAND}" &

 
# sleep 20
# gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && rosrun minimalpnp mainMinimalpnp" ' &

# trap 'kill $(jobs -pr) && clear' SIGINT SIGTERM EXIT


# TODO add custom catkin_ws path, currently assuming it's in ~

# # change these paths if needed!
 ORB_SLAM_ROOT=/home/epflcvlab/programs/ORB_SLAM2

 #--disable-factory needed for getting pid of background terminal
 gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscore" ' &
 pid0=$!

 sleep 4

 gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscd usb_cam && cd launch/ && roslaunch usb_cam-test.launch " ' &
 pid1=$!

 COMMAND='/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${ORB_SLAM_INSTALL_ROOT}/Examples/ROS && cd ${ORB_SLAM_INSTALL_ROOT} && rosrun SMART_ORB_SLAM2 Mono Vocabulary/ORBvoc.txt ./Examples/ROS/SMART_ORB_SLAM2/SMART_ORBSLAM2_Logitec9000.yaml" '
 ORB_SLAM_INSTALL_ROOT=${ORB_SLAM_ROOT}  gnome-terminal --disable-factory -e "${COMMAND}" &
 pid2=$!

 
 sleep 5
gnome-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && rosrun minimalpnp mainMinimalpnp" ' &
pid3=$!

 sleep 5
gnome-terminal --disable-factory -e '/bin/bash -c "rosrun image_view image_view image:=/SMART_ORB_SLAM2/trackedImageWithBox" ' &
pid4=$!

trap 'kill -SIGTERM $(jobs -pr) && clear' SIGINT SIGTERM EXIT
# trap 'kill -SIGTERM $pid0 $pid1 $pid2 $pid3' SIGINT SIGTERM EXIT
#wait "$pid0"
#wait "$pid1"
#wait "$pid2"
#wait "$pid3"
