#pragma once
#include <stdexcept>
#include "base_path_search.h"

namespace srrg_core {

  class ClustererPathSearch: public BasePathSearch {
  public:
    struct Cluster{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      PathMapCell* cell;
      Eigen::Vector2i lower;
      Eigen::Vector2i upper;
      float mean_r;
      float mean_c;
      int point_count;
      cv::Vec3b color;
      Vector2iVector pixels;
    };
    typedef std::vector<Cluster, Eigen::aligned_allocator<Cluster> > ClusterVector;
    ClustererPathSearch();
    

    //! @param regions image: this is the input.
    // cell to cluster should be set to 0
    // each cell to ignore should be set to -1;
    inline void setRegionsImage(const IntImage& regions_image) {_regions_image=&regions_image;}

    void setColor(int color_);
    const cv::Vec3b& color(){return _color;}

    virtual void init() override; //< call this once after setting indices image
    
    virtual bool compute() override;

    // returns a vector of cluster centroids 
    inline const ClusterVector& clusters() const {return _clusters;}
    inline const RGBImage& clusterImage() const {return _cluster_image;}
    
  protected:
    void fillFromImage();
    void expandRegion(PathMapCell* cell);
    
    int _max_index;
    cv::Vec3b _color;
    const IntImage* _regions_image;
    RGBImage _cluster_image;
    ClusterVector _clusters;
  };

}
