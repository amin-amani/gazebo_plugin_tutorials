cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES}  ${roscpp_LIBRARIES})

add_library(roscomunication SHARED roscomunication.cc)
target_link_libraries(roscomunication ${GAZEBO_LIBRARIES}  ${roscpp_LIBRARIES})



add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(surena SHARED surena.cc)
target_link_libraries(surena ${GAZEBO_LIBRARIES})
