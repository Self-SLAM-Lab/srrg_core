#include "path_map.h"
#include "clusterer_path_search.h"
#include "srrg_system_utils/system_utils.h"

using namespace std;

using namespace srrg_core;



int main(int argc, char** argv){

  cerr << "loading image from file: " << argv[1] << endl;
  cv::Mat image= cv::imread(argv[1]);
  int rows=image.rows;
  int cols=image.cols;
  
  UnsignedCharImage gray_image;
  cv::cvtColor(image, gray_image, CV_BGR2GRAY);

  cv::imshow("cluster_image",gray_image);
  cv::waitKey();

  IntImage regions;
  regions.create(rows, cols);
  for (int r=0;r<rows; ++r) {
    int* regions_ptr=regions.ptr<int>(r);
    const unsigned char* gray_ptr=gray_image.ptr<const unsigned char>(r);
    for (int c=0;c<cols; ++c, ++gray_ptr, ++ regions_ptr){
      *regions_ptr = (*gray_ptr>10) ? -1 : 0;
    }
  }
  ClustererPathSearch clusterer;
  PathMap  output_map;
  clusterer.setOutputPathMap(output_map);
  clusterer.setRegionsImage(regions);
  cerr << "init" << endl;
  clusterer.init();
  cerr << "compute" << endl;
  clusterer.compute();
  cerr << "done" << endl;
  const ClustererPathSearch::ClusterVector& clusters = clusterer.clusters();
  cerr << "found " << clusters.size() << " clusters" << endl;

  for (const ClustererPathSearch::Cluster cluster: clusters) {
    cv::circle(gray_image, cv::Point(cluster.mean_c, cluster.mean_r), 3, cv::Scalar(125)); 
  }

  cv::imshow("cluster_image",gray_image);
  cv::waitKey();
  return 0;
}
