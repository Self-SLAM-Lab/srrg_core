#include <iostream>
#include <string.h>
#include "srrg_system_utils/system_utils.h"
#include "srrg_image_utils/bresenham.h"

using namespace std;
using namespace srrg_core;


const char* banner[] = {
  "srrg_bresenham_example_app: example on how to run bresenham line traversal algorithm",
  "",
  "usage: srrg_bresenham_example_app <dump_file>",
  0
};

int main(int argc, char ** argv) {
//  if (argc < 2 || !strcmp(argv[1], "-h")) {
//    printBanner(banner);
//    return 0;
//  }

  Eigen::Vector2i start = Eigen::Vector2i(0,0);
  Eigen::Vector2i end = Eigen::Vector2i(0,10);
  Vector2iVector points;

  Bresenham::line(points,start,end);
  cerr << "Traversed " << points.size() << " grid cells!" << endl;
  for(int i=0; i < points.size(); i++){
      cerr << "x: " << points[i].x() << " - y: " << points[i].y() << endl;
  }
  cerr << endl << "done" << endl;
}
