cmake_minimum_required(VERSION 3.8)
project(rrt_v2)

set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
    STRING "Choose the type of build." FORCE)
endif()


set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Wunused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(Eigen3)
find_package(utils REQUIRED)

include_directories(include)

if(Eigen3_FOUND)
  set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
  find_package(Eigen)
  set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()


set(SOURCE_FILES
  src/rrt.cpp
)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  utils
)


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  include/
  DESTINATION include)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
