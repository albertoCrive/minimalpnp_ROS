cmake_minimum_required(VERSION 2.8.3)
project(minimalpnp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport)
find_package(OpenCV)

########## PoseEstimator dependencies ################
##### TODO : add them to the package.xml

# TODO for some reason these package are exported with full path in PoseEstimatorConfig.cmake, so no need to find package .. to be investigated,
###############################################
# find_package(OpenGL REQUIRED)		      #
# include_directories(${OpenGL_INCLUDE_DIRS}) #
# link_directories(${OpenGL_LIBRARY_DIRS})    #
# add_definitions(${OpenGL_DEFINITIONS})      #
# 					      #
   			      #
# find_package(GLEW REQUIRED)		      #
# include_directories(${GLEW_INCLUDE_DIRS})   #
# link_directories(${GLEW_LIBRARY_DIRS})      #
# 					      #
# pkg_check_modules(EIGEN3 REQUIRED eigen3 )  #
###############################################

find_package( PkgConfig REQUIRED)	    
pkg_search_module(GLFW REQUIRED glfw3)      
include_directories(${GLFW_INCLUDE_DIRS})   
link_directories(${GLFW_LIBRARY_DIRS})      

FIND_PACKAGE(PoseEstimator REQUIRED)
include_directories(${PoseEstimator_INCLUDE_DIRS})
link_directories(${PoseEstimator_LIB_DIR})


if (";${PoseEstimator_LIBS};" MATCHES ";afcuda;")
  MESSAGE(STATUS "PoseEstimator depends on Array fire and cuda. Will try to include them :  ${PoseEstimator_LIBS}")
  set(USE_AF 1)
else ()
  MESSAGE(STATUS "PoseEstimator does NOT depend on Array fire nor cuda  : ${PoseEstimator_LIBS}")
  set(USE_AF 0)
endif()

set(AF_LIBS "")

IF(USE_AF)
  FIND_PACKAGE(CUDA REQUIRED)
  FIND_PACKAGE(ArrayFire REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCNN_FAST_TEST")
  MESSAGE(STATUS "Compiling with fast array fire CNN testing!!!")
  include_directories(${ArrayFire_INCLUDE_DIRS})
  FIND_LIBRARY(CUDA_NVVM_LIBRARY NAMES "nvvm" PATH_SUFFIXES "nvvm/lib64" "nvvm/lib" PATHS ${CUDA_TOOLKIT_ROOT_DIR} DOC "CUDA NVVM Library")
  MESSAGE(STATUS "CUDA-ARRAYFIRE ENABLED.")
  set(AF_LIBS ${ArrayFire_CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES};${CUDA_LIBRARIES};${CUDA_cusolver_LIBRARY};${CUDA_CUFFT_LIBRARIES};${CUDA_NVVM_LIBRARY};${CUDA_CUDA_LIBRARY};${ARRAYFIRE_LIBRARIES};)
else()
  MESSAGE(STATUS "ATTENTION CUDA-ARRAY FIRE NOT ENABLED ! !! ")
endif()

set(POSE_ESTIMATOR_LIBS ${PoseEstimator_LIBS} ${OpenCV_LIBRARIES} ${AF_LIBS})



##########  END OF PoseEstimator dependencies ################




## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES minimalpnp
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(minimalpnp
#   src/${PROJECT_NAME}/minimalpnp.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(minimalpnp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(minimalpnp_node src/minimalpnp_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(minimalpnp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(minimalpnp_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS minimalpnp minimalpnp_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_minimalpnp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=c++11 ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wextra -Wno-sign-compare -DNDEBUG -DEIGEN_NO_DEBUG ") 
IF(UNIX)
	IF (NOT APPLE)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DISLINUX -Wfatal-errors")
	else()
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DISMAC")
	endif()
endif()

include_directories(${OPENCV_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_RANSAC")




add_executable(mainMinimalpnp main.cpp)
target_link_libraries(mainMinimalpnp ${POSE_ESTIMATOR_LIBS} ${catkin_LIBRARIES})
