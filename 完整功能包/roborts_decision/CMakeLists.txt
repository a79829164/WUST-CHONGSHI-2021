project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()

#blue_master
add_executable(blue_master
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/wait_behavior.h
  example_behavior/ready_behavior.h
  example_behavior/observe_behavior.h
  example_behavior/swing_behavior.h
  example_behavior/F1_behavior.h
  example_behavior/F2_behavior.h
  example_behavior/F3_behavior.h
  example_behavior/F4_behavior.h
  example_behavior/F5_behavior.h
  example_behavior/F6_behavior.h
  Blue_1.cpp
  executor/gimbal_executor.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(blue_master
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(blue_master
  roborts_msgs_generate_messages)

#blue_wing
add_executable(blue_wing
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/wait_behavior.h
  example_behavior/ready_behavior.h
  example_behavior/observe_behavior.h
  example_behavior/swing_behavior.h
  example_behavior/F1_behavior.h
  example_behavior/F2_behavior.h
  example_behavior/F3_behavior.h
  example_behavior/F4_behavior.h
  example_behavior/F5_behavior.h
  example_behavior/F6_behavior.h
  Blue_2.cpp
  executor/gimbal_executor.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(blue_wing
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(blue_wing
  roborts_msgs_generate_messages)

#-------------------------------------------RED-------------------------------------------#
#red_master
add_executable(red_master
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/wait_behavior.h
  example_behavior/ready_behavior.h
  example_behavior/observe_behavior.h
  example_behavior/swing_behavior.h
  example_behavior/F1_behavior.h
  example_behavior/F2_behavior.h
  example_behavior/F3_behavior.h
  example_behavior/F4_behavior.h
  example_behavior/F5_behavior.h
  example_behavior/F6_behavior.h
  Red_1.cpp
  executor/gimbal_executor.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(red_master
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(red_master
  roborts_msgs_generate_messages)

#red_wing
add_executable(red_wing
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/wait_behavior.h
  example_behavior/ready_behavior.h
  example_behavior/observe_behavior.h
  example_behavior/swing_behavior.h
  example_behavior/F1_behavior.h
  example_behavior/F2_behavior.h
  example_behavior/F3_behavior.h
  example_behavior/F4_behavior.h
  example_behavior/F5_behavior.h
  example_behavior/F6_behavior.h
  Red_2.cpp
  executor/gimbal_executor.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(red_wing
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(red_wing
  roborts_msgs_generate_messages)

#Roborts_test
add_executable(Roborts_Test
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/wait_behavior.h
  example_behavior/ready_behavior.h
  example_behavior/observe_behavior.h
  example_behavior/swing_behavior.h
  example_behavior/F1_behavior.h
  example_behavior/F2_behavior.h
  example_behavior/F3_behavior.h
  example_behavior/F4_behavior.h
  example_behavior/F5_behavior.h
  example_behavior/F6_behavior.h
  ROBORT_TEST.cpp
  executor/gimbal_executor.cpp
  executor/chassis_executor.cpp
  )

target_link_libraries(Roborts_Test
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(Roborts_Test
  roborts_msgs_generate_messages)

