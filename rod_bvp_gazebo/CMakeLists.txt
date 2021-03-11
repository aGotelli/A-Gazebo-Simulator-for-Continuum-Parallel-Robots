cmake_minimum_required(VERSION 3.0.2)
project(rod_bvp_gazebo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_msgs
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_path ( ZMQ_INCLUDE_DIR zmq.h )
find_library ( ZMQ_LIBRARY NAMES zmq )

catkin_package()


# common lib for message passing about shape description
add_library(rod_bvp_shape src/shape_listener.cpp src/shape_publisher.cpp)
target_include_directories(rod_bvp_shape PRIVATE
                                include ${ZMQ_INCLUDE_DIR})
target_link_libraries(rod_bvp_shape ${ZMQ_LIBRARY})

# visual plugin only depends on common lib
add_library(rod_bvp_visual_plugin src/visual_plugin.cpp)
target_include_directories(rod_bvp_visual_plugin PRIVATE include ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(rod_bvp_visual_plugin rod_bvp_shape ${GAZEBO_LIBRARY_DIRS})

# common lib for physics - only maths
add_library(rod_bvp_physics src/physics_core.cpp)
target_include_directories(rod_bvp_physics PRIVATE
                            include ${ZMQ_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(rod_bvp_physics rod_bvp_shape)

# physics as an external node
add_executable(rod_bvp_physics_node src/physics_node.cpp)
target_include_directories(rod_bvp_physics_node PRIVATE
                    include ${ZMQ_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(rod_bvp_physics_node rod_bvp_physics ${catkin_LIBRARIES})
