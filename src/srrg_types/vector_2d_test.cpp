#include "vector_2d.h"

using namespace std;
using namespace srrg_core;


typedef Vector2D<float> FloatVector2D;

int main (int argc, char ** argv) {
  int rows=100;
  int cols=200;
  
  FloatVector2D grid;

  grid.resize(rows, cols);
  int k=0;
  for (size_t r=0; r<grid.rows(); ++r) {
    for (size_t c=0; c<grid.cols(); ++c){
      grid.at(r,c)=k;
      k++;
    }
  }

  FloatVector2D other_grid=grid;

}
