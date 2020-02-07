cmake_minimum_required(VERSION 3.5)
project(refrain_my_quadrotor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(my_quadrotor my_quadrotor.cpp)
ament_target_dependencies(my_quadrotor rclcpp std_msgs)

if (${gazebo_VERSION_MAJOR} LESS 6)
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(my_quadrotor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(my_quadrotor ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES})
endif()

install(TARGETS
  my_quadrotor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
