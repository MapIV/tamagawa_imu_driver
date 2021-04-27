cmake_minimum_required(VERSION 3.5)
project(tamagawa_imu_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(tag_serial_driver src/tag_serial_driver.cpp)

ament_auto_add_executable(tag_can_driver src/tag_can_driver.cpp)

install(TARGETS
tag_serial_driver
  DESTINATION lib/tag_serial_driver
)

install(TARGETS
  tag_can_driver
  DESTINATION lib/tag_can_driver
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

