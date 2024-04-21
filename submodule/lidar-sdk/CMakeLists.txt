cmake_minimum_required(VERSION 2.8.3)
project(bigbang_rplidar_sdk)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_CXX_STANDARD 11)

FILE(GLOB RPLIDAR_SDK_SRC 
  sdk/src/arch/linux/*.cpp
  sdk/src/hal/*.cpp
  sdk/src/*.cpp
)

add_library(bigbang_rplidar_sdk STATIC ${RPLIDAR_SDK_SRC})
target_include_directories(bigbang_rplidar_sdk PUBLIC
  sdk/include
  sdk/src
)
