#pragma once
#include "srrg_types/defs.h"
#include "srrg_types/vector_2d.h"
#include <deque>
#include <queue>
#include <set>
#include <limits>

namespace srrg_core {

  /*!Cell of a distance map.
     Each cell has a row, a column, a parent (which is the closest occupied cell), and a distance (which is a distance to the cell).
   */
  struct PathMapCell{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //! ctor
    PathMapCell(){
      parent = 0;
      distance = std::numeric_limits<int>::max();
      r = 0;
      c = 0;
    }
    //! row and column
    int r, c;
    //! the closest occupied cell
    PathMapCell* parent;
    //! the distance to the closest
    float distance;
    //! weight factor for the distance (smaller for nearer objects);
    float weight;

  };

  /*! Distance map structure
    It is a grid of PathMapCells, stored as a vector_2d
    A Distance map is a grid that for each cell contains: the distance from the closest occupied cell AND the identity of the closest cell

    The typical use is the following:


    \code{.cpp}
    occupied_cells = ...;// vector of pairs denoting the occupied cells
    // prepare an int image and set all occupied cells
    IntImage input_indices(rows,cols);
    input_indices=-1; // clean the int image
    for(size_t i=0; i<occupied_cells.size(); i++){ 
       const std::pair<int,int> occupied_cell=occupied_cells[i];
       input_indics.at<int>(occupied_cell->first, occupied_cell->second)=i // mark the points in the occupied cells;
    }
    PathMap dmap;
    FloatImage distances;
    IntImage   closest_point_indices;

    dmap.compute(closest_point_indices, distances, input_indices, max_distance);

    
    \endcode
    here closest_point_indices is an int_image that contains in the cell [i,j] the index
    (meaning the position in the array) closest to the location i,j, while 
    distances[i,j] contains the distance from the point at i,j and the closest occupied cell.
    max_distance is the squared distance where to stop the expansion
  */
  class PathMap : public Vector2D<PathMapCell>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


    //! makes an inage out of a dmap
    void toImage(UnsignedCharImage& img) const;

    void fill(float distance, float weight);
  };


  //! Queue entry to compute the distance map from an IntImage
  //! It is exposed here to support the construction of different sorts of visit
  //! algorithms, such as Dijkstra or A*

  struct QEntry{
    QEntry(PathMapCell* c=0, float d=std::numeric_limits<float>::max()) {
      cell = c;
      distance = d;
    }

    //! comparison operator, returns the closest cell
    inline bool operator < (const QEntry& e) const {
      return e.distance < distance ;
    }

    float distance;
    PathMapCell* cell;
  };
  
  //! priority queue of PathMapCells, supports several algorithms
  //! it is a sorted container that keeps the entries ordered bu the distance
  struct PathMapCellQueue : public std::priority_queue<QEntry> {
    typedef typename std::priority_queue<QEntry>::size_type size_type;
    PathMapCellQueue(size_type capacity = 0) { reserve(capacity); };
    inline void reserve(size_type capacity) { this->c.reserve(capacity); } 
    inline size_type capacity() const { return this->c.capacity(); } 
    //! returns the top element of the queue;
    inline PathMapCell* top() { return std::priority_queue<QEntry>::top().cell;}
    //! pushes an element in the priority queue
    inline void push(PathMapCell* c) { return std::priority_queue<QEntry>::push(QEntry(c, c->distance));}
  };



}
