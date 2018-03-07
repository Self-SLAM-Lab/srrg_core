add_executable(srrg_message_converter_kitti_app srrg_message_converter_kitti_app.cpp)
target_link_libraries(srrg_message_converter_kitti_app
  srrg_system_utils_library
  srrg_messages_library
)

add_executable(srrg_message_converter_euroc_app srrg_message_converter_euroc_app.cpp)
target_link_libraries(srrg_message_converter_euroc_app
  srrg_system_utils_library
  srrg_messages_library
)

add_executable(srrg_message_sorter_app srrg_message_sorter_app.cpp)
target_link_libraries(srrg_message_sorter_app
  srrg_system_utils_library
  srrg_messages_library
)

add_executable(srrg_kitti_evaluate_odometry_app srrg_kitti_evaluate_odometry_app.cpp)
target_link_libraries(srrg_kitti_evaluate_odometry_app
srrg_system_utils_library
)

#ds visual tracking example: processes a sequence of images and attempts to track features between each subsequent pair in the sequence
add_executable(visual_tracker visual_tracker.cpp)
target_link_libraries(visual_tracker ${OpenCV_LIBS})

add_executable(visual_tracker_optical_flow visual_tracker_optical_flow.cpp)
target_link_libraries(visual_tracker_optical_flow ${OpenCV_LIBS})

#ds visual stereo feature matching example: processes a single pair of stereo images and attempts to find epipolar correspondences
add_executable(triangulator_stereo triangulator_stereo.cpp)
target_link_libraries(triangulator_stereo ${OpenCV_LIBS})

#ds visual, monocular triangulation example for a sequence of images (KITTI dataset)
add_executable(triangulator_mono triangulator_mono.cpp)
target_link_libraries(triangulator_mono srrg_messages_library ${OpenCV_LIBS})
