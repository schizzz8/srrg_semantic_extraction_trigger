add_library(srrg_semantic_extraction_trigger_library SHARED
  cloud_generator.cpp cloud_generator.h
  structure_analyzer.cpp structure_analyzer.h
  clusters_extractor.cpp clusters_extractor.h
  semantic_extraction_trigger.cpp semantic_extraction_trigger.h
)
target_link_libraries(srrg_semantic_extraction_trigger_library
  ${catkin_LIBRARIES}
)
