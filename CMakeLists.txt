cmake_minimum_required(VERSION 3.0.2)
project(mimree_description)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  robot_state_publisher
  roscpp
  rospy
  tf
  urdf
  geometry_msgs
  sensor_msgs
  message_filters
  laser_geometry
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS gazebo_ros gazebo_ros_control robot_state_publisher roscpp rospy tf urdf geometry_msgs sensor_msgs message_filters laser_geometry
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(laser_to_pc src/mimree_description/laser_to_pc.cpp)
add_dependencies(laser_to_pc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_to_pc ${catkin_LIBRARIES})

add_executable(mimree_tf_broadcaster src/mimree_description/mimree_tf_broadcaster.cpp)
add_dependencies(mimree_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mimree_tf_broadcaster ${catkin_LIBRARIES})

add_executable(edge_wt_detector src/mimree_description/edge_wt_detector.cpp)
add_dependencies(edge_wt_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(edge_wt_detector ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(TARGETS edge_wt_detector 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


install(TARGETS mimree_tf_broadcaster 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(TARGETS laser_to_pc 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
