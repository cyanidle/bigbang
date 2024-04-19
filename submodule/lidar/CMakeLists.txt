cmake_minimum_required(VERSION 2.8.3)
project(bigbang_rplidar_sdk)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_STANDARD 11)

FILE(GLOB RPLIDAR_SDK_SRC 
  ${CMAKE_SOURCE_DIR}/sdk/src/arch/linux/*.cpp
  ${CMAKE_SOURCE_DIR}/sdk/src/hal/*.cpp
  ${CMAKE_SOURCE_DIR}/sdk/src/*.cpp
)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rosconsole
  sensor_msgs
)

add_library(bigbang_rplidar_sdk STATIC ${RPLIDAR_SDK_SRC})
target_link_libraries(bigbang_rplidar_sdk PRIVATE ${catkin_LIBRARIES})
target_include_directories(bigbang_rplidar_sdk PUBLIC
  ${CMAKE_SOURCE_DIR}/sdk/include
  ${CMAKE_SOURCE_DIR}/sdk/src
  ${catkin_INCLUDE_DIRS}
)
