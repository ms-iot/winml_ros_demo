cmake_minimum_required(VERSION 2.8.3)
project(winml_ros_demo)

## Compile as at least C++17 for cppwinrt
add_compile_options(/Od)
add_compile_options(/std:c++latest)
add_compile_options(/await)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  interactive_markers
  roscpp
  roslib
  std_msgs
  visualization_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES winml_ros_demo
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport interactive_markers roscpp roslib std_msgs visualization_msgs
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/demo.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_fake src/ml_fake.cpp)
target_link_libraries(${PROJECT_NAME}_fake ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_fake
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)