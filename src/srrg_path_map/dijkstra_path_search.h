#include <stdexcept>
#include "base_path_search.h"

namespace srrg_core {

  class DijkstraPathSearch: public BasePathSearch {
  public:
    DijkstraPathSearch();
    
    //! 
    inline float costMax() const {return _cost_max;}
    inline void setCostMax(float cost_max_) { _cost_max=cost_max_; }

    inline void setCostFactorProportional(float cost) {_cost_factor_proportional=cost;}
    inline void setCostFactorDifferential(float cost) {_cost_factor_differential=cost;}
    

    inline float cellTraversalCost() const { return _cell_traversal_cost; }

    inline void setCellTraversalCost(float cell_traversal_cost) {
      _cell_traversal_cost = cell_traversal_cost;
      _cell_traversal_cost_diagonal = std::sqrt(2.0f) *_cell_traversal_cost;
    }

    //! @param sets a map that contains the cost of being in a particular cell. This is the input.
    inline void setCostMap(const FloatImage& cost_map) {_cost_map=&cost_map;}

    inline Vector2iVector& goals() {return _goals;}

    virtual bool compute() override;

  protected:
    float _cost_max; // in pixels, maximum cost for a cell(used to avoid obstacles)
    float _cost_factor_proportional;
    float _cost_factor_differential;
    float _cell_traversal_cost, _cell_traversal_cost_diagonal; // 
    const FloatImage* _cost_map;
    Vector2iVector _goals;
  };

}
