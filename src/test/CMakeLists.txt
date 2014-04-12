cmake_minimum_required(VERSION 2.8.3)
project(test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

# Find catkin
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	gazebo_ros 
) 

# Find Boost
find_package(Boost REQUIRED COMPONENTS 
	system 
	filesystem 
) 

# Find Gazebo
find_package(gazebo REQUIRED COMPONENTS)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(CEGUI CEGUI)
  pkg_check_modules(CEGUI-OGRE CEGUI-OGRE)
endif()

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
  DEPENDS 
    roscpp
    gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR} 
	${GPSTK_INCLUDE_DIR} 
	${GEOGRAPHICLIB_INCLUDE_DIR} 
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
	${GEOGRAPHICLIB_LIBRARY_DIRS} 
	${SDFormat_LIBRARY_DIRS} 
	${GAZEBO_LIBRARY_DIRS} 
	${OGRE_LIBRARY_DIRS}
  	${OGRE-Terrain_LIBRARY_DIRS}
  	${CEGUI_LIBRARY_DIRS}
  	${CEGUI-OGRE_LIBRARY_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}/src/msgs
)

## GUI / system plugins
add_library(test  		  			
	src/test.cpp
)

## GUI plugin
target_link_libraries(test       	
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES} 
	${Boost_LIBRARIES}
	${OGRE_LIBRARIES}
	${OGRE-Terrain_LIBRARIES}
	${CEGUI_LIBRARIES}
	${CEGUI-OGRE_LIBRARIES}
)

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
# install(TARGETS test test_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
