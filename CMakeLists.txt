cmake_minimum_required(VERSION 3.0.2)
project(tamagawa_imu_driver)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  diagnostic_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  can_msgs
  diagnostic_msgs
  diagnostic_updater
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(tag_serial_driver src/tag_serial_driver.cpp)
target_link_libraries(tag_serial_driver ${catkin_LIBRARIES})

add_executable(tag_can_driver src/tag_can_driver.cpp)
target_link_libraries(tag_can_driver ${catkin_LIBRARIES})

install(TARGETS tag_serial_driver tag_can_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS
  tag_serial_driver
  tag_can_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
