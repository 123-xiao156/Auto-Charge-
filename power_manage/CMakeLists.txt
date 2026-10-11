# #############################################################################
# CMake
# #############################################################################
cmake_minimum_required(VERSION 3.10.0)
project(power_manage)

# #############################################################################
# Catkin
# #############################################################################
SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
set(CMAKE_CXX_FLAGS "-std=c++14 -g -O0 ${CMAKE_CXX_FLAGS}")

# 调试选项定位到行
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-inline")

find_package(Qt5 COMPONENTS Core Widgets PrintSupport Network SerialPort REQUIRED)
find_package(Qt5SerialPort)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin COMPONENTS REQUIRED
  tf
  roscpp
  std_msgs
  pcl_conversions
  rosmsg
  roslib
  geometry_msgs
  nav_msgs
  image_transport
  sensor_msgs
  robot_msgs
)

set(QT_LIBRARIES Qt5::Widgets Qt5::SerialPort)

include_directories(${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  include)
catkin_package()
include_directories(/usr/local/openssl/include)
link_directories(
  /usr/local/openssl/lib/
)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.h include/*.hpp)
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
add_executable(${PROJECT_NAME}_node ${QT_SOURCES} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME}_node ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::Core ${OpenCV_LIBS} ssl crypto ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so.0.7.0)

# 打包开放 配置要开放的文件夹（安装后可在/opt/ros/kinetic/share/${PROJECT_NAME}路径下找到）
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
  
cmake_policy(SET CMP0081 NEW)
