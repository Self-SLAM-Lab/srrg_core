#include "distance_map_path_search.h"
#include <iostream>
namespace srrg_core {
  using namespace std;
  
  DistanceMapPathSearch::DistanceMapPathSearch() {
    setMaxDistance(10);
    _indices_image=0;
  }


  bool DistanceMapPathSearch::compute() {
    if (! _output_path_map)
      throw std::runtime_error("no output map selected");
    
    if (! _indices_image)
      throw std::runtime_error("no indices_image selected");
    
    int rows = _indices_image->rows;
    int cols = _indices_image->cols;
    if(rows<3 || cols<3){
      throw std::runtime_error("map too small to compute distance map");
    }

    _indices_map.create(rows,cols);
    _indices_map = -1;

    PathMap& output=*_output_path_map;
    output.resize(rows,cols);
    output.fill(_max_squared_distance, 1.0f);

    
    PathMapCellQueue q;//(rows*cols);
    //q.reserve(rows*cols);
    for (int r=0; r<rows; ++r){
      const int* index_ptr=_indices_image->ptr<const int>(r);
      int* index_map_ptr=_indices_map.ptr<int>(r);
      PathMapCell* cell_ptr=output.rowPtr(r);
      for (int c=0; c<cols; ++c, ++index_ptr, ++cell_ptr, ++index_map_ptr){
	int idx = *index_ptr;
	if (idx>-1){
	  *index_map_ptr=idx;
	  cell_ptr->parent = cell_ptr;
	  cell_ptr->distance = 0;
	  q.push(cell_ptr);
	}
      }
    }
    int operations = 0;
    size_t maxQSize = q.size();
    // cerr << "startq: "  << maxQSize << endl;
    //int currentDistance = 0;

    while (! q.empty()){
      PathMapCell* current = q.top();
      PathMapCell* parent = current->parent;
      int parent_index = _indices_map.at<int>(parent->r, parent->c);
      q.pop();

      if (output.onBorder(current->r, current->c))
	continue;
      for (int i=0; i<8; i++){
	PathMapCell* children=  current+output.eightNeighborOffsets()[i];
	int r = children->r;
	int c = children->c;
	int dr = r-parent->r;
	int dc = c-parent->c;
	int d=(dr*dr+dc*dc);
	// // cerr << "children: " << children->r << " "  << children->c << " " << children->distance <<  " " << d << endl;
	operations++;
	if (d<_max_squared_distance && children->distance>d) {
	  children->parent = parent;
	  _indices_map.at<int>(r,c) = parent_index;
	  children->distance = d;
	  q.push(children);
	}
      }
      maxQSize = maxQSize < q.size() ? q.size() : maxQSize;
    }
    //double t1 = getTime();
    //cerr << "# operations: " << operations << " maxQ: " << maxQSize << " time: " << t1-t0 << endl;

    
    _distance_image.create(rows,cols);
    for (int r=0; r<rows; ++r){
      PathMapCell* cell_ptr=output.rowPtr(r);
      const int* index_ptr=_indices_image->ptr<const int>(r);
      float* distance_ptr=_distance_image.ptr<float>(r);
      for (int c=0; c<cols; ++c, ++cell_ptr, ++index_ptr, ++distance_ptr){
	float d = std::sqrt(cell_ptr->distance);
	if (d!=d) {
	  cerr << "mcell.distance: " << cell_ptr->distance << endl;
	  throw std::runtime_error("nan in dmap calculation");
	}
	const int& idx=*index_ptr;
	if ( idx < -1)
	  d=-d;
 	*distance_ptr=d;
      }
    }

    return true;
  }

}
