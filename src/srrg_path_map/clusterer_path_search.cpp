#include "clusterer_path_search.h"

#include <iostream>
namespace srrg_core {
using namespace std;

ClustererPathSearch::ClustererPathSearch() {
    _regions_image=0;
}


void ClustererPathSearch::init() {
    if (! _output_path_map)
        throw std::runtime_error("no output map selected");
    
    if (! _regions_image)
        throw std::runtime_error("no regions_image selected");
    
    int rows = _regions_image->rows;
    int cols = _regions_image->cols;
    if(rows<3 || cols<3){
        throw std::runtime_error("map too small to compute clustrer map");
    }

    PathMap& output=*_output_path_map;
    output.resize(rows,cols);
    output.fill(std::numeric_limits<float>::max(), 1.0f);
    fillFromImage();
    _clusters.clear();
    _max_index=0;

}

void ClustererPathSearch::fillFromImage() {
    PathMap& output=*_output_path_map;
    int rows = output.rows();
    int cols = output.cols();
    for (int r=0; r<rows; ++r){
        const int* regions_ptr=_regions_image->ptr<const int>(r);
        PathMapCell* cell_ptr=output.rowPtr(r);
        for (int c=0; c<cols; ++c, ++regions_ptr, ++cell_ptr){
            int idx = *regions_ptr;
            if (!idx) {
                cell_ptr->cost=1;
            } else {
                cell_ptr->cost=std::numeric_limits<float>::max();
            }
        }
    }
}

void ClustererPathSearch::expandRegion(PathMapCell* cell) {
    //cerr << __PRETTY_FUNCTION__ << ": r=" <<cell->r << " " << "c=" << cell->c << endl;
    PathMap& output=*_output_path_map;
    int rows = output.rows();
    int cols = output.cols();

    int r_min=std::numeric_limits<int>::max(),r_max=std::numeric_limits<int>::min();
    int c_min=std::numeric_limits<int>::max(),c_max=std::numeric_limits<int>::min();

    // cerr << "startq: "  << maxQSize << endl;
    //int currentDistance = 0;
    cell->parent=cell;
    cell->distance=0;
    std::deque<PathMapCell*> queue;
    queue.push_back(cell);
    int r_sum=cell->r;
    int c_sum=cell->c;

    if(cell->r < r_min)
        r_min = cell->r;
    if(cell->r > r_max)
        r_max = cell->r;

    if(cell->c < c_min)
        c_min = cell->c;
    if(cell->c > c_max)
        c_max = cell->c;

    int count=1;
    while (! queue.empty()){
        PathMapCell* current = queue.front();
        queue.pop_front();
        if (output.onBorder(current->r, current->c))
            continue;
        for (int i=0; i<8; i++){
            PathMapCell* child=  current+output.eightNeighborOffsets()[i];
            if(child->cost==std::numeric_limits<float>::max())
                continue;
            if (child->parent)
                continue;
            child->parent=cell;
            r_sum+=child->r;
            c_sum+=child->c;
            if(child->r < r_min)
                r_min = child->r;
            if(child->r > r_max)
                r_max = child->r;

            if(child->c < c_min)
                c_min = child->c;
            if(child->c > c_max)
                c_max = child->c;
            ++count;
            _num_operations++;
            queue.push_back(child);
        }
    }
    Cluster cluster;
    cluster.cell=cell;
    cluster.mean_r=(float)r_sum/(float) count;
    cluster.mean_c=(float)c_sum/(float) count;
    cluster.lower=Eigen::Vector2i(r_min,c_min);
    cluster.upper=Eigen::Vector2i(r_max,c_max);
    cluster.point_count=count;
    _clusters.push_back(cluster);
    ++_max_index;
}


bool ClustererPathSearch::compute() {
    PathMap& output=*_output_path_map;
    int rows = output.rows();
    int cols = output.cols();

    _num_operations = 0;
    for (int r=0; r<rows; ++r){
        PathMapCell* cell_ptr=output.rowPtr(r);
        for (int c=0; c<cols; ++c, ++cell_ptr){
            if (output.onBorder(r, c))
                continue;
            if (cell_ptr->cost==std::numeric_limits<float>::max())
                continue;
            if (cell_ptr->parent)
                continue;
            expandRegion(cell_ptr);
        }
    }
    return true;
}

}
