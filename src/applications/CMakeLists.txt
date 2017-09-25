add_executable(srrg_message_converter_kitti_app srrg_message_converter_kitti_app.cpp)
target_link_libraries(srrg_message_converter_kitti_app
  srrg_system_utils_library
  srrg_messages_library
  ${Boost_LIBRARIES}
)

add_executable(srrg_message_converter_euroc_app srrg_message_converter_euroc_app.cpp)
target_link_libraries(srrg_message_converter_euroc_app
  srrg_system_utils_library
  srrg_messages_library
  ${Boost_LIBRARIES}
)

add_executable(srrg_message_sorter_app srrg_message_sorter_app.cpp)
target_link_libraries(srrg_message_sorter_app
  srrg_system_utils_library
  srrg_messages_library
  ${Boost_LIBRARIES}
)

add_executable(srrg_kitti_evaluate_odometry_app srrg_kitti_evaluate_odometry_app.cpp)
target_link_libraries(srrg_kitti_evaluate_odometry_app
srrg_system_utils_library
)

#ds visual feature tracking example: processes a sequence of images and attempts to track features between each subsequent pair in the sequence
add_executable(visual_feature_tracker visual_feature_tracker.cpp)
target_link_libraries(visual_feature_tracker ${OpenCV_LIBS})
