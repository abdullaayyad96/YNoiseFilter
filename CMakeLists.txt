cmake_minimum_required(VERSION 2.8.3)
project(ynoise_filter)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(Boost REQUIRED COMPONENTS system thread)

# make the executable
cs_add_executable(ynoise_node
  src/ynoise_node.cpp
  src/ynoise.cpp
)


# link the executable to the necesarry libs
target_link_libraries(ynoise_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Install the nodelet library
cs_install()
