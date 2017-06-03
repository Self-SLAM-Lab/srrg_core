#pragma once

#include <srrg_types/defs.h>

#include <srrg_types/cloud.h>

namespace srrg_nicp {

  //! shrinks a depth image, by a shrink factor k
  //! k=2 means the resulting image is half size
  //! k must be a perfect divider of the input
  //! in dest the region shrunk is interpolated
  void shrinkRawDepth(srrg_core::RawDepthImage& dest_buffer, const srrg_core::RawDepthImage& src_buffer, int shrink_factor);

  //! shrinks a depth image, ad the indices by a shrink factor k
  //! k=2 means the resulting image is half size
  //! k must be a perfect divider of the input
  void shrinkDepth(srrg_core::FloatImage& dest_buffer, srrg_core::IntImage& dest_indices, 
		   const srrg_core::FloatImage& src_buffer, const srrg_core::IntImage& src_indices, int shrink_factor);

  void compareDepths(float& in_distance, 
		     int& in_num,
		     float& out_distance, 
		     int& out_num,
		     const srrg_core::FloatImage& depths1, const srrg_core::IntImage& indices1, 
		     const srrg_core::FloatImage& depths2, const srrg_core::IntImage& indices2, 
		     float dist=0.05, bool scale_z = false, srrg_core::FloatImage* result=0);


  //! does the merge of src in dest
  //! it requires the index image of dest and of src, seen from the same point
  //! and also the depth buffers
  //! the clouds should be aligned
  //! points that are closer than distanceThreshold are merged based on the
  //scaling values
  //! if the normals are compatible
  void merge(srrg_core::FloatImage& destBuffer, srrg_core::IndexImage& destIndices, srrg_core::Cloud& dest,
	     srrg_core::FloatImage& srcBuffer, srrg_core::IndexImage& srcIndices, srrg_core::Cloud& src,
	     float normalThreshold = 1, float distanceThreshold = 0.2);

  
  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, float scale = 1000.0f);
  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale = 1000.0f);
  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, float scale = 0.001f);
  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale = 0.001f);

  // [Workaround: OpenCV 3.0.0 image subtraction warnings]
  void add_16UC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);
  void sub_16UC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);
  void add_32FC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);
  void sub_32FC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2);

}
