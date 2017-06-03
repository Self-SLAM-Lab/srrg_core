add_library(srrg_image_utils_library
  depth_utils.cpp depth_utils.h
  point_image_utils.cpp point_image_utils.h
  )

target_link_libraries(srrg_image_utils_library
  srrg_core_types_library
  ${catkin_LIBRARIES}
  )
  
