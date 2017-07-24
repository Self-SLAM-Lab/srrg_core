add_library(srrg_path_map_library SHARED
  path_map.cpp path_map.h
  path_map_utils.cpp path_map_utils.h
  base_path_search.cpp base_path_search.h
  distance_map_path_search.cpp distance_map_path_search.h
  dijkstra_path_search.cpp dijkstra_path_search.h
  clusterer_path_search.cpp clusterer_path_search.h
)

target_link_libraries(srrg_path_map_library
  ${OpenCV_LIBS}
)

add_executable(distance_map_test distance_map_test.cpp)

target_link_libraries(distance_map_test
  srrg_path_map_library
  srrg_system_utils_library
  ${OPENCV_LIBS}
  )

add_executable(dijkstra_test dijkstra_test.cpp)

target_link_libraries(dijkstra_test
  srrg_path_map_library
  srrg_system_utils_library
  ${OPENCV_LIBS}
  )

add_executable(clusterer_test clusterer_test.cpp)

target_link_libraries(clusterer_test
  srrg_path_map_library
  srrg_system_utils_library
  ${OPENCV_LIBS}
  )
