# Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)
project(bs)

# Add custom cmake
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Find catkin
find_package(catkin REQUIRED COMPONENTS 
	roslib
  roscpp
	std_msgs
  std_srvs
	geometry_msgs
	message_generation
) 

# Find Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(TinyXML tinyxml2)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(CEGUI CEGUI)
  pkg_check_modules(CEGUI-OGRE CEGUI-OGRE)
endif()

# Find Boost
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
) 

# Find protocol buffers
find_package(Protobuf REQUIRED)

# Find GPS toolkit
find_package(GPSTK REQUIRED)

# Find GPS toolkit
find_package(GeographicLib REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


#############################################
## Declare Gazebo protocol buffer messages ##
#############################################

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# Generate the message source and header files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS 
  src/msgs/meteorological.proto 
  src/msgs/environment.proto 
  src/msgs/satellites.proto
  src/msgs/wind.proto
)

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
add_message_files(
  DIRECTORY 
    msg
  FILES
    Collision.msg
    Collisions.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY 
    srv
  FILES
    Insert.srv
    Delete.srv
    Step.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
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
    LIBRARIES
      bs_resources
      bs_experiment
      bs_simulation
      bs_environment
      bs_meteorological
      bs_satellites
      bs_wind
      bs_altimeter
      bs_compass
      bs_energy
      bs_gnss
      bs_imu
      bs_propulsion
      bs_shear
      bs_turbulence
    CATKIN_DEPENDS
      roslib
      roscpp
      std_msgs
      std_srvs
      message_generation
      message_runtime
      rosgraph_msgs
      geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR} 
	${SDFormat_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${GPSTK_INCLUDE_DIR} 
  ${GEOGRAPHICLIB_INCLUDE_DIR} 
  ${PROTOBUF_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
	${OGRE-Terrain_INCLUDE_DIRS}
	${CEGUI_INCLUDE_DIRS}
	${CEGUI-OGRE_INCLUDE_DIRS}
)

## Specify additional locations of library files
link_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
	${catkin_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS} 
	${SDFormat_LIBRARY_DIRS}
  ${TinyXML_LIBRARY_DIRS}
  ${GPSTK_LIBRARY_DIRS}
  ${GEOGRAPHICLIB_LIBRARY_DIRS} 
  ${PROTOBUF_LIBRARY_DIRS}
	${GAZEBO_LIBRARY_DIRS} 
	${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${CEGUI_LIBRARY_DIRS}
  ${CEGUI-OGRE_LIBRARY_DIRS}
)

################################
# Define libraries to be built #

# Compile the message library
add_library(custom_msgs ${PROTO_SRCS})

# The base system plugins gzclient and gzserver
add_library(bs_resources       src/plugins/system/Resources.cpp)
add_library(bs_experiment      src/plugins/system/Experiment.cpp)
add_library(bs_simulation      src/plugins/system/Simulation.cpp)

# The world plugins
add_library(bs_environment     src/plugins/world/Environment.cpp)
add_library(bs_meteorological  src/plugins/world/Meteorological.cpp)
add_library(bs_satellites      src/plugins/world/Satellites.cpp)
add_library(bs_wind            src/plugins/world/Wind.cpp)

# The model plugins
add_library(bs_altimeter       src/plugins/model/Altimeter.cpp)
add_library(bs_compass         src/plugins/model/Compass.cpp)
add_library(bs_energy          src/plugins/model/Energy.cpp)
add_library(bs_gnss            src/plugins/model/GNSS.cpp)
add_library(bs_imu             src/plugins/model/IMU.cpp)
add_library(bs_quadrotor       src/plugins/model/Quadrotor.cpp)
add_library(bs_shear           src/plugins/model/Shear.cpp)
add_library(bs_turbulence      src/plugins/model/Turbulence.cpp)

##################################
## Add cmake target dependencies #

add_dependencies(bs_resources        bs_generate_messages_cpp)
add_dependencies(bs_experiment       bs_generate_messages_cpp)
add_dependencies(bs_simulation       bs_generate_messages_cpp)

add_dependencies(bs_environment      custom_msgs)
add_dependencies(bs_meteorological   custom_msgs)
add_dependencies(bs_satellites       custom_msgs)
add_dependencies(bs_wind             custom_msgs)
add_dependencies(bs_altimeter        custom_msgs)
add_dependencies(bs_compass          custom_msgs)
add_dependencies(bs_energy           custom_msgs)
add_dependencies(bs_gnss             custom_msgs)
add_dependencies(bs_imu              custom_msgs)
add_dependencies(bs_quadrotor        custom_msgs)
add_dependencies(bs_shear            custom_msgs)
add_dependencies(bs_turbulence       custom_msgs)

###########################
## Set up dynamic linking #

## Convenience declarations
set(SYS_LIBS ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES})
set(WLD_LIBS ${SYS_LIBS} ${GPSTK_LIBRARIES} ${GEOGRAPHICLIB_LIBRARY} custom_msgs)
set(MOD_LIBS ${SYS_LIBS} ${GPSTK_LIBRARIES} ${GEOGRAPHICLIB_LIBRARY} custom_msgs)

# Link the message library
target_link_libraries(custom_msgs        ${PROTOBUF_LIBRARY})

## System plugins
target_link_libraries(bs_resources       ${SYS_LIBS})
target_link_libraries(bs_experiment      ${SYS_LIBS})
target_link_libraries(bs_simulation      ${SYS_LIBS})

## System plugins
target_link_libraries(bs_environment     ${WLD_LIBS})
target_link_libraries(bs_meteorological  ${WLD_LIBS})
target_link_libraries(bs_satellites      ${WLD_LIBS})
target_link_libraries(bs_wind            ${WLD_LIBS})

# Model plugins
target_link_libraries(bs_altimeter       ${MOD_LIBS})
target_link_libraries(bs_compass         ${MOD_LIBS})
target_link_libraries(bs_energy          ${MOD_LIBS})
target_link_libraries(bs_gnss            ${MOD_LIBS})
target_link_libraries(bs_imu             ${MOD_LIBS})
target_link_libraries(bs_quadrotor       ${MOD_LIBS})
target_link_libraries(bs_shear           ${MOD_LIBS})
target_link_libraries(bs_turbulence      ${MOD_LIBS})

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