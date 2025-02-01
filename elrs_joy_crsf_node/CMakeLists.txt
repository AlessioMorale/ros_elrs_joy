cmake_minimum_required(VERSION 3.8)
project(elrs_joy_crsf_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(NODE_NAME crsf_node)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(elrs_joy_crsf_protocol REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  PRIVATE
  include
)

file(GLOB CRSF_NODE_SRC_FILES src/*.cpp)

add_executable(
  ${NODE_NAME}
  ${CRSF_NODE_SRC_FILES}
)

ament_target_dependencies(${NODE_NAME}
rclcpp rclcpp_lifecycle elrs_joy_crsf_protocol sensor_msgs diagnostic_msgs diagnostic_updater)

# install(
#   TARGETS
#   ${NODE_NAME}
#   RUNTIME DESTINATION bin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
# )

install(TARGETS ${NODE_NAME}
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
endif()

ament_package()
