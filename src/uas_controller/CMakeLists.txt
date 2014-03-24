# Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)
project(uas_controller)

# Add custom cmake
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Find catkin
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	gazebo_ros 
	uas_hal
) 

# Find Boost
find_package(Boost REQUIRED COMPONENTS 
	system 
	filesystem 
) 

# Find Gazebo
find_package(gazebo REQUIRED COMPONENTS)

# Find Protobuf
find_package(Protobuf REQUIRED)

# Find GPS toolkit
find_package(GPSTK REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(CEGUI CEGUI)
  pkg_check_modules(CEGUI-OGRE CEGUI-OGRE)
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
#    MessageTest.msg
# )

## Generate services in the 'srv' folder
add_service_files(
    FILES
	    Pause.srv
	    Reset.srv
	    Resume.srv
	    Step.srv
	    Insert.srv
	    Remove.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		std_msgs  # Or other packages containing msgs
)

## Generate gazebo messages
add_subdirectory(src/msgs)

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
  DEPENDS 
    roscpp
    gazebo_ros
    uas_hal
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR} 
	${GPSTK_INCLUDE_DIR} 
	${SDFormat_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
  	${OGRE-Terrain_INCLUDE_DIRS}
  	${CEGUI_INCLUDE_DIRS}
  	${CEGUI-OGRE_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}/src/msgs
)

## Specify additional locations of library files
link_directories(
	${catkin_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS} 
	${GPSTK_LIBRARY_DIRS}
	${SDFormat_LIBRARY_DIRS} 
	${GAZEBO_LIBRARY_DIRS} 
	${OGRE_LIBRARY_DIRS}
  	${OGRE-Terrain_LIBRARY_DIRS}
  	${CEGUI_LIBRARY_DIRS}
  	${CEGUI-OGRE_LIBRARY_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}/src/msgs
)

################################
# Define libraries to be built #

## GUI / system plugins
add_library(gui  		  			src/gui/gui.cpp)

## World plugins
add_library(simulation  			src/world/simulation.cpp)
add_library(experiment 				src/world/experiment.cpp)

## Model plugins
add_library(altimeter 				src/model/altimeter.cpp)
add_library(imu 					src/model/imu.cpp)
add_library(compass 				src/model/compass.cpp)
add_library(gpsreceiver 			src/model/gpsreceiver.cpp)
add_library(inclinometer 			src/model/inclinometer.cpp)
add_library(quadrotor 				src/model/quadrotor.cpp
	src/model/components/dynamics.cpp
	src/model/components/turbulence.cpp
	src/model/components/shear.cpp
	src/model/components/energy.cpp
	src/model/components/control.cpp
)

##################################
## Add cmake target dependencies #

## Model plugins
add_dependencies(quadrotor			custom_msgs)
add_dependencies(altimeter			custom_msgs)
add_dependencies(imu 				custom_msgs)
add_dependencies(compass 			custom_msgs)
add_dependencies(inclinometer 		custom_msgs)
add_dependencies(gpsreceiver 		custom_msgs)

## World plugins
add_dependencies(simulation 		custom_msgs)
add_dependencies(experiment			custom_msgs)

###########################
## Set up dynamic linking #

## GUI plugins
target_link_libraries(gui       	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_LIBRARIES} ${OGRE-Terrain_LIBRARIES} ${CEGUI_LIBRARIES} ${CEGUI-OGRE_LIBRARIES})

## World plugins
target_link_libraries(simulation	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)
target_link_libraries(experiment	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)

## Model plugins
target_link_libraries(quadrotor		${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)
target_link_libraries(altimeter 	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)
target_link_libraries(imu 	 		${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)
target_link_libraries(compass 		${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)
target_link_libraries(gpsreceiver  	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)
target_link_libraries(inclinometer 	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES} custom_msgs)


## Declare a cpp executable
add_executable(gpstest
	src/gpstest.cpp
)
target_link_libraries(gpstest	 	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${GPSTK_LIBRARIES})

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
# install(TARGETS uav_msgs uav_msgs_node
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
# install(
# 	DIRECTORY
# 		launch
# 	DESTINATION 
# 		${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uav_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)