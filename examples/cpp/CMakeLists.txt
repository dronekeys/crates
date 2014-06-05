## Minimum cmake version
cmake_minimum_required(VERSION 2.8.3)

## Project name
project(example)

## Link to required simulator
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  sim
)

## Include CRATES libraries
include_directories( 
  ${catkin_INCLUDE_DIRS}
)

## The example executable
add_executable(
  example main.cpp
)

## Link against CRATES libraries
target_link_libraries(example 
  ${catkin_LIBRARIES}
)