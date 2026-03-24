cmake_minimum_required(VERSION 3.8)
project(engineai_ros2_node_template)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  interface_protocol
)

foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

add_executable(${PROJECT_NAME}
  src/engineai_ros2_node_template.cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
