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

find_package(catkin REQUIRED COMPONENTS
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

add_subdirectory(include/lidar)

catkin_install_python(PROGRAMS
  nodes/arduino_interface.py
  nodes/local_planer.py
  nodes/task_manager.py
  nodes/interconnect.py
  nodes/scripts_executor.py
  nodes/gui.py
  # ^ Place node scripts here
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(Qt5 REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-sign-compare -O3 -fPIC -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -Wno-sign-compare")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pthread")
set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS}")
set(CMAKE_AUTOMOC ON)

add_library(Types STATIC
  include/bigbang/types/coord.hpp
  include/bigbang/types/position.hpp
  include/bigbang/types/costmap.hpp
  include/bigbang/types/rosparams.cpp
  include/bigbang/types/serializer.hpp #needs to be added, so moc gets linked
)
target_include_directories(Types PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${radapter_broker_INCLUDE_DIRS}
)
target_link_libraries(Types PUBLIC
  describe pcg
  ${catkin_LIBRARIES}
)
add_library(nodebase STATIC
  include/bigbang/nodes/nodebase.cpp
)
target_link_libraries(nodebase PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Qt5::Core
  Types
)


add_executable(costmap_server
  include/bigbang/nodes/costmapservernode.h
  src/costmapservernode.cpp
)
target_link_libraries(costmap_server PRIVATE nodebase)

add_executable(monte_carlo
  include/bigbang/utils/measurement_utils.hpp
  include/bigbang/utils/particles.hpp
  include/bigbang/nodes/monte_carlo.h
  src/monte_carlo.cpp
)
target_link_libraries(monte_carlo PRIVATE nodebase)

add_executable(global_planer
  include/bigbang/nodes/globalplanernode.h
  src/globalplanernode.cpp
)
target_link_libraries(global_planer PRIVATE nodebase)

add_executable(bigbang_rplidar
  include/bigbang/utils/beacons_shape.hpp
  include/bigbang/utils/rplidar_utils.h
  include/bigbang/utils/rplidar_utils.cpp
  include/bigbang/nodes/rplidarnode.h
  src/rplidarnode.cpp
)

target_link_libraries(bigbang_rplidar PRIVATE nodebase bigbang_rplidar_sdk)

install(DIRECTORY launch nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
