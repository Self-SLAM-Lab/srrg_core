#pragma once
#include "distance_map.h"

namespace srrg_core {
  class BasePathSearch{
  public:
    BasePathSearch();
    virtual ~BasePathSearch() = 0;
    virtual void setup(); // to set the parameters after construction

    inline void setOutputPathMap(PathMap& output) {_output_path_map=&output;}
    inline const PathMap& outputPathMap() const {return *_output_path_map;}
    inline  PathMap& outputPathMap() {return *_output_path_map;}
    
    virtual void init();        // to be called before compute when all parameters are in place
    virtual bool compute()=0; // to do the actual calculation. Return false on unreachable goal. Output written on pathmap;
  protected:
    PathMap *_output_path_map;
  };

}
