cmake_minimum_required(VERSION 2.8.3)
project(estrella2)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hacer estrella2.cpp)

target_link_libraries(hacer ${catkin_LIBRARIES})