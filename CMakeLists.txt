cmake_minimum_required(VERSION 2.8.3)
project(tamagawa_imu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tag_driver src/tag_driver.cpp)
target_link_libraries(tag_driver ${catkin_LIBRARIES})
