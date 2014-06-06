# Mikimum cmake version
cmake_minimum_required(VERSION 2.8.3)

# Project name
project(example)

# We need rosjava to buidl this project
find_package(catkin REQUIRED rosjava_build_tools)

# Ability to install apps and publish to shared repository
catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)

# Project defined by a catkin package
catkin_package()

# Shared folders
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/crates/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/crates/${PROJECT_NAME})
