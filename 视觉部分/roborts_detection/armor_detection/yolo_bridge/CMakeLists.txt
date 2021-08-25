project(yolo_bridge)


add_library(yolo_bridge
  SHARED
  ${YoloBridgeProtoSrc}
  ${YoloBridgeProtoHds}
  yolo_bridge.cpp
  )

add_library(detection::yolo_bridge ALIAS yolo_bridge)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  detection::tool
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
