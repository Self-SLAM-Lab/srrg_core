add_executable(srrg_txt_io_synchronizer_example srrg_txt_io_synchronizer_example.cpp)
target_link_libraries(srrg_txt_io_synchronizer_example
  srrg_system_utils_library
  srrg_core_types_library
  srrg_txt_io_library
  ${catkin_LIBRARIES}
)

add_executable(srrg_open_file_example_app srrg_open_file_example_app.cpp)
target_link_libraries(srrg_open_file_example_app
  srrg_system_utils_library
  srrg_core_types_library
  srrg_txt_io_library
  ${catkin_LIBRARIES}
)

add_executable(srrg_open_boss_file_example_app srrg_open_boss_file_example_app.cpp)
target_link_libraries(srrg_open_boss_file_example_app
  srrg_txt_io_library
  srrg_system_utils_library
  srrg_core_types_library
  srrg_boss_library
  ${catkin_LIBRARIES}
)


add_executable(srrg_trajectory_saver_example srrg_trajectory_saver_example.cpp)
target_link_libraries(srrg_trajectory_saver_example
  srrg_system_utils_library
  srrg_core_types_library
  srrg_txt_io_library
  ${catkin_LIBRARIES}
)

add_executable(srrg_kdtree_example srrg_kdtree_example.cpp)
target_link_libraries(srrg_kdtree_example
  srrg_system_utils_library
)

add_executable(srrg_trajectory_saver_example_app srrg_trajectory_saver_example_app.cpp)
target_link_libraries(srrg_trajectory_saver_example_app
  srrg_system_utils_library
  srrg_core_types_library
  srrg_txt_io_library
  ${catkin_LIBRARIES}
)



add_executable(srrg_remove_odometry_offset_example_app srrg_remove_odometry_offset_example_app.cpp)
target_link_libraries(srrg_remove_odometry_offset_example_app
  srrg_system_utils_library
  srrg_core_types_library
  srrg_txt_io_library
  ${catkin_LIBRARIES}
)

