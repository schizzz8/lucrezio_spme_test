add_executable(semantic_mapper_app
  semantic_mapper_app.cpp
)

target_link_libraries(semantic_mapper_app
  semantic_mapper_library
  semantic_explorer_library
  map_evaluator_library
  ${catkin_LIBRARIES}
)

