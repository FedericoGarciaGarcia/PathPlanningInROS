cmake_minimum_required(VERSION 2.8.3)
project(my_stage)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services

## Generate messages in the 'msg' folder
# add_message_files(FILES MyMessage.msg)

## Generate services in the 'srv' folder
# add_service_files(FILES MyService.srv)

## Generate added messages and services 
# generate_messages(DEPENDENCIES std_msgs)

## Declare catkin package
catkin_package()

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(move_forward src/move_forward.cpp)
add_executable(stopper src/Stopper.cpp src/run_stopper.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(move_forward ${catkin_LIBRARIES})
target_link_libraries(stopper ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)
