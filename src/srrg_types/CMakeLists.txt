add_library(srrg_core_types_library
  cloud.cpp cloud.h
  image_data.cpp image_data.h
  )

target_link_libraries(srrg_core_types_library
  srrg_boss_library
  )
