cmake_minimum_required(VERSION 2.8.3)
project(asl)

set(OpenCV_DIR ~/openCV/opencv/release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  kdl_conversions
  openni2_camera
  roscpp
  roslib
  message_generation
  std_msgs
  pcl_conversions
  pcl_ros
  tf
  OpenCV
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES skeleton_tracker
  CATKIN_DEPENDS cv_bridge image_transport kdl_conversions openni2_camera  roscpp roslib message_runtime std_msgs pcl_conversions pcl_ros tf
  DEPENDS OpenCV
)

set(OPENNI2_DIR ~/OpenNI2)
set(OPENNI2_WRAPPER /opt/ros/kinetic/include/openni2_camera/)
set(NITE2_DIR ~/OpenNI2/NiTE-Linux-x64-2.2/)
set(NITE2_LIB ~/OpenNI2/NiTE-Linux-x64-2.2/Redist/libNiTE2.so)

link_directories(${OPENNI2_DIR}/Bin/x64-Release)

include_directories(${OPENNI2_DIR}/Bin/x64-Release)
include_directories(/usr/include/openni2/)
include_directories(${OPENNI2_DIR}/Include)
include_directories(${OPENNI2_WRAPPER})
include_directories(${NITE2_DIR}/Include)
include_directories(${OpenCV_INCLUDE_DIRS}/include)

add_executable(hand_tracker src/hand_tracker.cpp src/classes/HandTracker.cpp src/classes/Sensor.cpp)
target_link_libraries(hand_tracker OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(sample_generator src/sample_generators/sample_generator.cpp src/classes/HandTracker.cpp src/classes/Sensor.cpp src/classes/Preprocessor.cpp src/classes/Viewer.cpp)
target_link_libraries(sample_generator OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ASL src/main.cpp src/classes/HandTracker.cpp src/classes/Sensor.cpp src/classes/Preprocessor.cpp)
target_link_libraries(ASL OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# No longer used because it caused overfitting during training. Kept for reference.
#add_executable(sample_generator_small src/sample_generators/sample_generator_small.cpp)
#target_link_libraries(sample_generator_small ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
