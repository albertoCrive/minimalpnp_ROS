#!/bin/bash


#### change these paths if necessary
ORB_SLAM_ROOT=/home/epflcvlab/programs/ORB_SLAM2
CURRENT_TERMINAL=mate-terminal
CATKIN_WS_PATH="/home/epflcvlab/catkin_ws"


### you should not change anything below here ! 

ROSCORE_LAUNCH='/bin/bash -c "cd ${CATKIN_WS} && source devel/setup.bash && roscore" '

USB_CAM_LAUNCH='/bin/bash -c "sleep 5 && cd ${CATKIN_WS} && source devel/setup.bash && roscd usb_cam && cd launch/ && roslaunch usb_cam-test.launch " '

MINIMALPNP_ROS_LAUNCH='/bin/bash -c "cd ${CATKIN_WS} && source devel/setup.bash && rosrun minimalpnp mainMinimalpnp" '

ORBSLAM_LAUNCH='/bin/bash -c "cd ${CATKIN_WS} && source devel/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${ORB_SLAM_INSTALL_ROOT}/Examples/ROS && cd ${ORB_SLAM_INSTALL_ROOT} && rosrun SMART_ORB_SLAM2 Mono Vocabulary/ORBvoc.txt ./Examples/ROS/SMART_ORB_SLAM2/SMART_ORBSLAM2_Logitec9000.yaml" '

VIEWER_LAUNCH='/bin/bash -c "sleep 10 && rosrun image_view image_view image:=/SMART_ORB_SLAM2/trackedImageWithBox autosize:=true" '


#--disable-factory needed for getting pid of background terminal
ORB_SLAM_INSTALL_ROOT=${ORB_SLAM_ROOT} CATKIN_WS=${CATKIN_WS_PATH} ${CURRENT_TERMINAL} --disable-factory --tab -e "${ROSCORE_LAUNCH}" \
              --tab -e "${USB_CAM_LAUNCH}" \
	      --tab -e  "${ORBSLAM_LAUNCH}" \
              --tab -e "${MINIMALPNP_ROS_LAUNCH}" \
              --tab -e "${VIEWER_LAUNCH}" &

trap 'kill $(jobs -pr) && clear' SIGINT SIGTERM EXIT




# TODO add custom catkin_ws path, currently assuming it's in ~

# # change these paths if needed!

 #--disable-factory needed for getting pid of background terminal
# mate-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscore" ' &

# pid0=$!
#  sleep 4

#  mate-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && roscd usb_cam && cd launch/ && roslaunch usb_cam-test.launch " ' &
#  pid1=$!

# COMMAND='/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${ORB_SLAM_INSTALL_ROOT}/Examples/ROS && cd ${ORB_SLAM_INSTALL_ROOT} && rosrun SMART_ORB_SLAM2 Mono Vocabulary/ORBvoc.txt ./Examples/ROS/SMART_ORB_SLAM2/SMART_ORBSLAM2_Logitec9000.yaml" '
#  ORB_SLAM_INSTALL_ROOT=${ORB_SLAM_ROOT}  mate-terminal --disable-factory -e "${COMMAND}" &
#  pid2=$!

 
#  sleep 5
# mate-terminal --disable-factory -e '/bin/bash -c "cd ~/catkin_ws && source devel/setup.bash && rosrun minimalpnp mainMinimalpnp" ' &
# pid3=$!

# sleep 5
# mate-terminal --disable-factory -e '/bin/bash -c "rosrun image_view image_view image:=/SMART_ORB_SLAM2/trackedImageWithBox" ' &
# pid4=$!

# trap 'kill $(jobs -pr) && clear' SIGINT SIGTERM EXIT
# # trap 'kill -SIGTERM $pid0 $pid1 $pid2 $pid3' SIGINT SIGTERM EXIT
# #wait "$pid0"
# #wait "$pid1"
# #wait "$pid2"
# #wait "$pid3"
