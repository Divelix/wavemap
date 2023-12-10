cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros_conversions)

# find_package(wavemap REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../wavemap ${CMAKE_CURRENT_BINARY_DIR}/wavemap)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    eigen_conversions
    wavemap_msgs
)

catkin_package()

add_library(wavemap_ros_conversions
  src/config_conversions.cc
  src/map_msg_conversions.cc
  src/time_conversions.cc
)
target_compile_features(wavemap_ros_conversions PUBLIC cxx_std_20)
target_include_directories(wavemap_ros_conversions
  PUBLIC
    include
    ../../../wavemap/wavemap_core/include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(wavemap_ros_conversions
  wavemap_core
  ${catkin_LIBRARIES}
)

install(TARGETS wavemap_ros_conversions RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})