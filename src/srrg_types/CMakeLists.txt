add_library(srrg_core_types_library
  cloud_3d.cpp cloud_3d.h
  image_data.cpp image_data.h
  )

target_link_libraries(srrg_core_types_library
  srrg_boss_library
  )
