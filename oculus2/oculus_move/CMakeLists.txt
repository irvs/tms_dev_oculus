cmake_minimum_required(VERSION 2.8.3)
project(oculus_move)

find_package(catkin REQUIRED COMPONENTS 
  cv_bridge
  image_geometry
  std_msgs
  genmsg
  roscpp
  rospy
  tf
  roscpp
  geometry_msgs
  tms_msg_db
)

find_package(ecto_opencv REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

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
#  LIBRARIES oculus_move
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs tms_msg_db
  DEPENDS system_lib
)

###########
## Build ##
###########

# Declare a cpp executable
 add_executable(oculus_move src/oculus_move.cpp)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
 add_dependencies(oculus_move keytf_generate_messages_cpp tms_msg_db_gencpp)

# Specify libraries to link a library or executable target against
 target_link_libraries(oculus_move
   ${catkin_LIBRARIES} mysqlclient
 )

add_executable(oculus_robot_motion src/oculus_robot_motion.cpp)
add_dependencies(oculus_robot_motion tms_msg_rc_gencpp)
target_link_libraries(oculus_robot_motion ${catkin_LIBRARIES} omniORB4 omnithread)

add_executable(oculus_real_vision src/oculus_real_vision.cc)
#add_dependencies(oculus_real_vision)
target_link_libraries(oculus_real_vision ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
