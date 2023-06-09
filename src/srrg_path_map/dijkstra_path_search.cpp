#include <stdexcept>
#include "dijkstra_path_search.h"

namespace srrg_core {

DijkstraPathSearch::DijkstraPathSearch() {
    _cost_map=0;
    setCellTraversalCost(1.0f);
    setCostMax(100);
    _cost_factor_proportional=1;
    _cost_factor_differential=0;
}

bool DijkstraPathSearch::compute(){

    if (!_cost_map)
        throw std::runtime_error("cost map not set, not possible to compute minimal paths");

    if (!_output_path_map)
        throw std::runtime_error("output map not set, not possible to compute minimal paths");

    PathMap& output = *_output_path_map;
    const int rows=_cost_map->rows;
    const int cols=_cost_map->cols;
    output.resize(rows, cols);

    for (size_t r=0; r<rows; ++r){
        PathMapCell* cell_ptr=output.rowPtr(r);
        const float* cost_ptr=_cost_map->ptr<const float>(r);
        for (size_t c=0; c<cols; ++c, ++cell_ptr, ++cost_ptr){
            cell_ptr->r=r;
            cell_ptr->c=c;
            cell_ptr->parent=0;
            cell_ptr->distance=std::numeric_limits<float>::max();
            cell_ptr->cost=*cost_ptr;
        }
    }

    // prepare the frontier for expansion
    PathMapCellQueue q;//(rows*cols);

    for (const Eigen::Vector2i& goal: _goals){
        const int r=goal.x();
        const int c=goal.y();
        if (!output.inside(r,c)){
            return false;
        }
        if (_cost_map->at<float>(r,c) > _cost_max){
            return false;
        }
        PathMapCell& goal_cell=output(r,c);
        goal_cell.distance=0;
        goal_cell.parent=&goal_cell;
        q.push(&goal_cell);
    }

    _num_operations=0;
    size_t max_q_size=q.size();
    float current_distance=0;
    // pull and expand
    float d_min=0;
    while (! q.empty()){
        PathMapCell* current = q.top();
        q.pop();
        if (current->distance<d_min)
            continue;
        d_min=current->distance;
        if (output.onBorder(current->r, current->c))
            continue;

        for (int i=0; i<8; i++){
            PathMapCell* child=  current+output.eightNeighborOffsets()[i];
            if (child->cost>_cost_max)
                continue;
            if (child->distance<current_distance)
                continue;
            current_distance=child->distance;

            int dr = child->r - current->r;
            int dc = child->c - current->c;
            float step_cost=(dr==0||dc==0) ? _cell_traversal_cost : _cell_traversal_cost_diagonal;

            // adjust the cost based on the difference between source and target

            float cost_difference=child->cost-current->cost;

            float adjusted_cost=std::max(_cost_factor_proportional,
                                         _cost_factor_proportional*child->cost+
                                         _cost_factor_differential*cost_difference);

            step_cost*=adjusted_cost;

            float estimated_distance=step_cost+current->distance;
            if (child->distance>estimated_distance) {
                child->parent = current;
                child->distance = estimated_distance;
                q.push(child);
            }
            _num_operations++;
        }
        if (q.size() > max_q_size)
            max_q_size = q.size();

    }
}

}
