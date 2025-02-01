cmake_minimum_required(VERSION 3.8)
project(elrs_joy_crsf_protocol CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  PUBLIC
  include
)

file(GLOB CRSF_SRC_FILES src/*.cpp)

add_library(
  ${PROJECT_NAME}
  ${CRSF_SRC_FILES}
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB TEST_SOURCES test/*.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TEST_SOURCES})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
