add_library(srrg_path_map_library SHARED
  path_map.cpp path_map.h
  base_path_search.cpp base_path_search.h
  distance_map_path_search.cpp distance_map_path_search.h
  dijkstra_path_search.cpp dijkstra_path_search.h
)

target_link_libraries(srrg_path_map_library
  ${OpenCV_LIBS}
)
