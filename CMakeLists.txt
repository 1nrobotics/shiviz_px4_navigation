cmake_minimum_required(VERSION 3.10.2)

project(px4_offboard)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  eigen_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES px4_offb
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs tf2 tf2_ros
#  DEPENDS system_lib
#   DEPENDS Eigen3
)
set(YAML_BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")

add_subdirectory(thirdparty/yaml-cpp)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/waypoint_navigator.cpp 
)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    # yaml-cpp::yaml-cpp
    yaml-cpp
    Threads::Threads
)

install(FILES config/waypoints.yaml DESTINATION ${CMAKE_BINARY_DIR}/config)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS yaml-cpp
    EXPORT yaml-cpp-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)