cmake_minimum_required(VERSION 3.8)
project(mission_director)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(mission_director src/mission_director.cpp)

include_directories(
  include
)

ament_target_dependencies(mission_director  
  rclcpp
  geometry_msgs
  std_msgs
  px4_msgs
)

# Library
add_library(mission_director_lib
  src/state.cpp
  src/state_disarmed.cpp
  src/state_takeoff.cpp
  src/state_land.cpp
  src/state_hover.cpp
  src/state_ats.cpp
)
target_include_directories(mission_director_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Executable
target_link_libraries(mission_director mission_director_lib)

install(TARGETS
  mission_director
  mission_director_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
