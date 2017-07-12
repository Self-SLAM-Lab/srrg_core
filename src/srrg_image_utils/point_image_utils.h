#pragma once
#include "srrg_types/defs.h"

namespace srrg_core {

  void initializePinholeDirections(Float3Image& directions_image,
				   const Eigen::Matrix3f& camera_matrix,
				   const UnsignedCharImage& mask=UnsignedCharImage());

  void initializeSphericalDirections(Float3Image& directions,
				     const Eigen::Matrix3f& camera_matrix,
				     const UnsignedCharImage& mask=UnsignedCharImage());

  
  void computePointsImage(Float3Image& point_image,
			  const Float3Image& direction_image,
			  const FloatImage&  depth_image,
			  const float min_distance,
			  const float max_distance);

  void computeSimpleNormals(Float3Image& normal_image,
			    const Float3Image& points_image,
			    int col_gap,
			    int row_gap,
			    float max_distance);


  void normalBlur(Float3Image& dest, const Float3Image& src, int window);
  
}
