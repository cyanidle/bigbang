cmake_minimum_required(VERSION 3.1.0)
project(bigbang_eurobot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(BUILD_SHARED_LIBS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(submodule/describe)
add_subdirectory(submodule/pcg)
add_subdirectory(submodule/lidar-sdk)

find_package(catkin COMPONENTS
  actionlib
  nav_msgs
  roscpp
  rospy
  tf
  std_msgs
  genmsg 
  rosserial_arduino
  rosserial_client
  sensor_msgs 
  message_generation
  rosconsole
  sensor_msgs
)

message(STATUS "Found catkin: ${catkin_FOUND}")

if (catkin_FOUND)
  catkin_python_setup()
  add_message_files(
      FILES
        ArduinoCommand.msg
        MotorInfo.msg
        MotorParams.msg
        ServoCreateUpdate.msg
        PinReader.msg
        PinReaderResponce.msg
        Move2d.msg
        Measure2d.msg
        Reconfigure.msg
        RawImu.msg
        ServoCommand.msg
        MapObject.msg
        PlanerStatus.msg
        LaserBeacon.msg
        LaserBeacons.msg
        MonteCarloState.msg
        ArduinoStatus.msg
        ArduinoLed.msg
    )
  
  add_service_files(
    FILES
      ExecuteScript.srv
      DirectMove.srv
      DirectDrift.srv
  )
  
  generate_messages(
    DEPENDENCIES
    std_msgs nav_msgs sensor_msgs actionlib_msgs
  )
  catkin_package(
    #INCLUDE_DIRS ${INCLUDE_DIRECTORIES}
    CATKIN_DEPENDS python3-numpy map_msgs geometry_msgs 
    nav_msgs roscpp rospy rosserial_arduino sensor_msgs 
    std_msgs message_runtime python3-pyyaml python3-asyncio
    #DEPENDS 
  )
  
  catkin_install_python(
    PROGRAMS
      nodes/arduino_interface.py
      nodes/local_planer.py
      nodes/task_manager.py
      nodes/scripts_executor.py
      nodes/gui.py
    DESTINATION 
      ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-sign-compare ")
set(CMAKE_AUTOMOC ON)

add_library(bigbang-common STATIC
  include/common/nodebase.hpp
  src/nodebase.cpp
  )

target_compile_options(bigbang-common PUBLIC
  -Wall -fPIC -Wno-sign-compare
)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  target_compile_options(bigbang-common PUBLIC
    -Ofast -ffast-math
  )
endif()

target_include_directories(bigbang-common PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(bigbang-common PUBLIC 
  BOOST_BIND_GLOBAL_PLACEHOLDERS
)

target_link_libraries(bigbang-common PUBLIC
  describe pcg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Qt5::Core
)

add_executable(costmap_server
  include/costmap/costmapservernode.hpp
  src/costmapservernode.cpp
)
target_link_libraries(costmap_server PRIVATE bigbang-common)

add_executable(monte_carlo
  include/monte_carlo/monte_carlo.hpp
  src/monte_carlo.cpp
)
target_link_libraries(monte_carlo PRIVATE bigbang-common)

add_executable(global_planer
  include/global_planer/globalplanernode.hpp
  src/globalplanernode.cpp
)
target_link_libraries(global_planer PRIVATE bigbang-common)

add_executable(bigbang_rplidar
  include/lidar/rplidarnode.hpp
  include/lidar/rplidar_utils.hpp
  src/rplidarnode.cpp
)

target_link_libraries(bigbang_rplidar PRIVATE bigbang-common bigbang-rplidar-sdk)

if (catkin_FOUND)
  install(DIRECTORY launch nodes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
  )
endif()
