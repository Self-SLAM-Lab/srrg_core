add_library(srrg_distance_map_library SHARED
  distance_map.cpp distance_map.h
  base_path_search.cpp base_path_search.h
  distance_map_path_search.cpp #distance_map_path_search.h
)

target_link_libraries(srrg_distance_map_library
  ${OpenCV_LIBS}
)
