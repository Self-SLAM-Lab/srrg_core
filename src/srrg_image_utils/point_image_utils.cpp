#include "point_image_utils.h"
#include <stdexcept>

namespace srrg_core {

  void initializePinholeDirections(Float3Image& directions,
				   const Eigen::Matrix3f& camera_matrix,
				   const UnsignedCharImage& mask){
    int rows=directions.rows;
    int cols=directions.cols;
    const Eigen::Matrix3f inverse_camera_matrix=camera_matrix.inverse();
    for (int r=0; r<rows; ++r) {
      cv::Vec3f* direction=directions.ptr<cv::Vec3f>(r);
      const unsigned char* masked=0;
      if (! mask.empty()) {
	masked=mask.ptr<const unsigned char>(r);
      }
      for (int c=0; c<cols; ++c, ++direction){
	*direction=cv::Vec3f(0.f,0.f,0.f);
	bool keep_point=(!masked || *masked);
	if (keep_point) {
	  Eigen::Vector3f dir=inverse_camera_matrix*Eigen::Vector3f(c,r,1);
	  *direction=cv::Vec3f(dir.x(), dir.y(), dir.z());
	} 
	if (masked) ++masked;
      }
    }
  }
  
  void computePointsImage(Float3Image& points_image,
			  const Float3Image& directions,
			  const FloatImage&  depth_image,
			  const float min_distance,
			  const float max_distance){
    if (directions.size()!=depth_image.size())
      throw std::runtime_error("directions and depth image sizes should match");
    int rows=directions.rows;
    int cols=directions.cols;
    points_image.create(rows, cols);
    for (int r=0; r<rows; ++r) {
      cv::Vec3f* point=points_image.ptr<cv::Vec3f>(r);
      const cv::Vec3f* direction=directions.ptr<const cv::Vec3f>(r);
      const float* depth=depth_image.ptr<const float>(r);
      for (int c=0; c<cols; ++c, ++direction, ++depth, ++point){
	float d=*depth;
	if (d>max_distance||d<min_distance)
	  d=0;
	*point=(*direction)*d;
      }
    }
  }
  
  void computeSimpleNormals(Float3Image& normal_image,
			    const Float3Image& points_image,
			    int col_gap,
			    int row_gap,
			    float max_distance) {

    normal_image.create(points_image.rows, points_image.cols);
    normal_image=cv::Vec3f(0.,0.,0.);
    float squared_max_distance=max_distance*max_distance;
    for (int r = row_gap; r < points_image.rows - row_gap; r++) {
      const cv::Vec3f* up_row_ptr=points_image.ptr<const cv::Vec3f>(r-row_gap) + col_gap;
      const cv::Vec3f* row_ptr=points_image.ptr<const cv::Vec3f>(r) + col_gap;
      const cv::Vec3f* down_row_ptr=points_image.ptr<const cv::Vec3f>(r+row_gap) + col_gap;
      cv::Vec3f* dest_row_ptr=normal_image.ptr<cv::Vec3f>(r) + col_gap;
      for (int c = col_gap;
	   c < points_image.cols - col_gap;
	     ++c, ++up_row_ptr, ++row_ptr, ++down_row_ptr, ++dest_row_ptr ) {
	
	*dest_row_ptr=cv::Vec3f(0.f, 0.f, 0.f);
	
	const cv::Vec3f& p = *row_ptr;
	// if z is null, skip; 
	if (p[2]==0)
	  continue;
      
	const cv::Vec3f& px00 = *(row_ptr-col_gap);
	if (px00[2]==0)
	  continue;
      
	const cv::Vec3f& px01 = *(row_ptr+col_gap);
	if (px01[2]==0)
	  continue;

	const cv::Vec3f& py00 = *up_row_ptr;
	if (py00[2]==0)
	  continue;

	const cv::Vec3f& py01 = *down_row_ptr;
	if (py01[2]==0)
	  continue;

	cv::Vec3f dx = px01 - px00;
	cv::Vec3f dy = py01 - py00;
	if (dx.dot(dx) > squared_max_distance ||
	    dy.dot(dy) > squared_max_distance) { continue; }

	cv::Vec3f n = dy.cross(dx);
	n = normalize(n);
	if (n.dot(p) > 0) { n = -n; }
	*dest_row_ptr = n;
      }
    }
  }

  void normalBlur(Float3Image& dest, const Float3Image& src, int window) {

    dest = Float3Image::zeros(src.size());
    int rows = dest.rows;
    int cols = dest.cols;
    Float3Image normal_integral;
    cv::integral(src, normal_integral, CV_32F);
    for (int r = window; r < rows - window; ++r ) {
      const cv::Vec3f* up_row_ptr=normal_integral.ptr<const cv::Vec3f>(r-window)+window;
      const cv::Vec3f* down_row_ptr=normal_integral.ptr<const cv::Vec3f>(r+window)+window;
      cv::Vec3f* dest_row_ptr=dest.ptr<cv::Vec3f>(r)+window;

      for (int c = window; c < cols - window;
	   ++c, ++down_row_ptr, ++up_row_ptr, ++dest_row_ptr) {
	cv::Vec3f m11=*(down_row_ptr+window);
	cv::Vec3f m00=*(up_row_ptr-window);
	cv::Vec3f m01=*(down_row_ptr-window);
	cv::Vec3f m10=*(up_row_ptr+window);
	cv::Vec3f n_sum=m11+m00-m01-m10;
	if (n_sum.dot(n_sum)>0.2)
	  *dest_row_ptr = normalize(n_sum);
	else
	  *dest_row_ptr = cv::Vec3f(0,0,0);
      }
    }
  }

}
