cmake_minimum_required(VERSION 3.0.2)
project(wam_teleop_datacollection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  wam_msgs
  geometry_msgs
  wam_srvs
)

## libbarrett
find_package(Barrett REQUIRED)
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## Boost
set(boost_components system thread python)
find_package(Boost REQUIRED ${boost_components})
link_directories(${Boost_LIBRARY_DIRS})

set(exported_include_dirs ${exported_include_dirs} ${Boost_INCLUDE_DIRS})
set(exported_link_dirs ${exported_link_dirs} ${Boost_LIBRARY_DIRS})

## curses
find_package(Curses REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BARRETT_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIR}
)


add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  wam_force_estimation
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  wam_msgs
  wam_srvs
)
