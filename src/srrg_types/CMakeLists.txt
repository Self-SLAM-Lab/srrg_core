add_library(srrg_core_types_library
  image_data.cpp image_data.h
  cloud_3d.cpp cloud_3d.h
  base_camera_info.cpp base_camera_info.h
  pinhole_camera_info.cpp pinhole_camera_info.h
  spherical_camera_info.cpp spherical_camera_info.h
  multi_camera_info.cpp multi_camera_info.h
  camera_info_manager.cpp camera_info_manager.h
  )

target_link_libraries(srrg_core_types_library
  srrg_boss_library
  ${OpenCV_LIBS}
  )
