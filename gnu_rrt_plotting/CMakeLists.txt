cmake_minimum_required(VERSION 3.8)
project(gnu_rrt_plotting)

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
find_package(Eigen3 REQUIRED)
find_package(Boost 1.74.0 COMPONENTS filesystem system iostreams program_options REQUIRED)
find_package(ompl REQUIRED)

if(Eigen3_FOUND)
  set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
  find_package(Eigen)
  set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()

include_directories(include)
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

set(SOURCE_FILES
  src/rrt_gnu_plotting.cpp
  src/main.cpp
)

# add_library(${PROJECT_NAME},
#   src/rrt_gnu_plotting.cpp
# )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)
ament_package()
