cmake_minimum_required(VERSION 3.8)
project(osiris_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(NODE_NAME obstacle_detector_2d )

add_library(${PROJECT_NAME} SHARED
  src/obstacle_detect.cpp
)

set(dependencies 
  rclcpp
  rclcpp_components

  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

include_directories(include)


ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

add_executable(${NODE_NAME} src/obstacle_detect_main.cpp)
target_link_libraries(${NODE_NAME} ${PROJECT_NAME})

install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${NODE_NAME} ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
