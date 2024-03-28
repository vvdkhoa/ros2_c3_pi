cmake_minimum_required(VERSION 3.0.2)
project(KAJIMA)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  ecl_threads
  genmsg
  roscpp
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
  cmake_modules
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)
find_package(PahoMqttCpp REQUIRED)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES KAJIMA
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
	DEPENDS
      TinyXML
)


###########
## Build ##
###########

find_library(wiringPi_LIB wiringPi)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(
	${PROJECT_NAME}_node

	src/KAJIMA_node.cpp
	src/MQTT_tool.cpp
    src/MQTT_tool.h
	src/ina228.h
	##src/ina228.c

   )


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}  
   ${wiringPi} ${wiringPi_LIB}  -lcrypt -lrt
   ${TinyXML_LIBRARIES}
   ${PAHO_MQTT_CPP_LIBRARIES}
   paho-mqtt3c paho-mqttpp3
 )
 