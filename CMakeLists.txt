cmake_minimum_required(VERSION 2.8.3)
project(can)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

##############
## Messages ##
##############

add_message_files(
  FILES
  CANFrame.msg
  Float32Stamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

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
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS message_runtime std_msgs
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(can_interface_in_node
  src/can_interface_in_node.cpp
  src/CANInterface.cpp
  )
add_executable(can_interface_out_node
  src/can_interface_out_node.cpp
  src/CANInterface.cpp
  )
add_executable(can_gateway_node
  src/can_gateway_node.cpp
  src/Signal.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(can_interface_in_node ${catkin_LIBRARIES})
target_link_libraries(can_interface_out_node ${catkin_LIBRARIES})
target_link_libraries(can_gateway_node ${catkin_LIBRARIES})

## First the messages/services/actions have to be built
add_dependencies(can_interface_in_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(can_interface_out_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(can_gateway_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS
  can_interface_in_node
  can_interface_out_node
  can_gateway_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
