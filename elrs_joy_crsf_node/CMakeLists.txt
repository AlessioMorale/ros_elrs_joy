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
find_package(ament_cmake_gtest QUIET)

include_directories(
  ${PROJECT_SOURCE_DIR}
  PRIVATE
  include
)

add_library(${PROJECT_NAME}_lib
  src/crsf_joy_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp rclcpp_lifecycle elrs_joy_crsf_protocol sensor_msgs diagnostic_msgs diagnostic_updater)

add_executable(${NODE_NAME}
  src/crsf_main.cpp
)

target_link_libraries(${NODE_NAME}
  ${PROJECT_NAME}_lib
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
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(${PROJECT_NAME}_test
      test/test_crsf_joy_node.cpp
    )
    if(TARGET ${PROJECT_NAME}_test)
      target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}_lib
      )
      ament_target_dependencies(${PROJECT_NAME}_test
        rclcpp
        sensor_msgs
        elrs_joy_crsf_protocol
      )
    endif()
  endif()
endif()

ament_package()
