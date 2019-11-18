cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
project(turtlebot_walker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  tf
)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  pcl
)

add_executable(main src/main.cpp src/walker.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(main turtlebot_walker_generate_messages_cpp ${turtlebot_walker_EXPORTED_TARGETS})
