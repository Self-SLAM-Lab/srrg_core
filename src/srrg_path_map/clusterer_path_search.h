#include <stdexcept>
#include "base_path_search.h"

namespace srrg_core {

  class ClustererPathSearch: public BasePathSearch {
  public:
    struct Cluster{
      PathMapCell* cell;
      int mean_r;
      int mean_c;
      int point_count;
    };
    typedef std::vector<Cluster> ClusterVector;
    ClustererPathSearch();
    

    //! @param regions image: this is the input.
    // cell to cluster should be set to 0
    // each cell to ignore should be set to -1;
    inline void setRegionsImage(const IntImage& regions_image) {_regions_image=&regions_image;}

    virtual void init() override; //< call this once after setting indices image
    
    virtual bool compute() override;

    // returns a vector of cluster centroids 
    inline const ClusterVector& clusters() const {return _clusters;}
    
  protected:
    void fillFromImage();
    void expandRegion(PathMapCell* cell);
    
    int _max_index;
    const IntImage* _regions_image;
    ClusterVector _clusters;
  };

}
