add_executable(data_dumper_node
  data_dumper_node.cpp
)

target_link_libraries(data_dumper_node
  ${catkin_LIBRARIES}
)

add_executable(map_viewer_node
  map_viewer_node.cpp
)

target_link_libraries(map_viewer_node
  map_evaluator_library
  ${catkin_LIBRARIES}
)

