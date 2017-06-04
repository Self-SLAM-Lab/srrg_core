#include "depth_utils.h"
#include <iostream>
#include <omp.h>
#include <stdexcept>

#define _NAN_CHECK_

namespace srrg_nicp {

  using namespace std;
  using namespace srrg_core;

  void shrinkRawDepth(RawDepthImage& dest_buffer, const RawDepthImage& src_buffer, int k){

    int rows = src_buffer.rows;
    int cols = src_buffer.cols;
    

    if (rows%k) {
      cerr << "shrinkRawDepth(...) : rows=" << rows << ", cols=" << cols << " k:" << k << endl;
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image rows");
    }

    if (cols%k) {
      cerr << "shrinkRawDepth(...) : rows=" << rows << ", cols=" << cols << " k:" << k << endl;
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image cols");
    }

    int drows = rows/k;
    int dcols = cols/k;


    dest_buffer.create(drows, dcols);
    dest_buffer=0;

    // avoid divisions and use a lookup table
    int lv = rows>cols?rows:cols;
    int ttable[lv];
    for (int i = 0; i<lv; i++)
      ttable[i] = i/k;

    for (int r = 0; r<rows; r++) {
      // get the row pointers of source and destination

      const unsigned short* src_z_ptr =src_buffer.ptr<unsigned short>(r);
      int dr = ttable[r];
      unsigned short* dest_z_ptr = dest_buffer.ptr<unsigned short>(dr);
      
      int cc = 0;
      for (int c = 0; c<cols; c++){
	unsigned short src_z = *src_z_ptr;
	src_z_ptr++;
	if (src_z==0)
	  continue;
	unsigned short& dest_z = *(dest_z_ptr+ttable[c]);
	if (! dest_z || dest_z<src_z)
	  dest_z = src_z;
      }
    }
  }


  void shrinkDepth(FloatImage& dest_buffer, IntImage& dest_indices, 
		   const FloatImage& src_buffer, const IntImage& src_indices, int k){

    int rows = src_buffer.rows;
    int cols = src_buffer.cols;
    
    if (src_buffer.rows!=src_indices.rows || src_buffer.cols!=src_indices.cols) {
      cerr << "shrinkDepth(...) : rows=" << rows << ", cols=" << cols << " k:" << k << endl;
      throw std::runtime_error("shrinkDepth: src indices and buffer do not match");
    }

    if (rows%k) {
      cerr << "shrinkDepth(...) : rows=" << rows << ", cols=" << cols << " k:" << k << endl;
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image rows");
    }

    if (cols%k) {
      cerr << "shrinkDepth(...) : rows=" << rows << ", cols=" << cols << " k:" << k << endl;
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image cols");
    }

    int drows = rows/k;
    int dcols = cols/k;

    const float big_value=std::numeric_limits<float>::max();

    dest_buffer.create(drows, dcols);
    dest_buffer=big_value;

    dest_indices.create(drows, dcols);
    dest_indices = -1;

    // avoid divisions and use a lookup table
    int lv = rows>cols?rows:cols;
    int ttable[lv];
    for (int i = 0; i<lv; i++)
      ttable[i] = i/k;

    for (int r = 0; r<rows; r++) {
      // get the row pointers of source and destination

      const float* src_z_ptr =src_buffer.ptr<float>(r);
      const int* src_idx_ptr=src_indices.ptr<int>(r);
      int dr = ttable[r];
      float* dest_z_ptr = dest_buffer.ptr<float>(dr);
      int* dest_idx_ptr = dest_indices.ptr<int>(dr);
      
      int cc = 0;
      for (int c = 0; c<cols; c++){
	float src_z = *src_z_ptr;
	int src_idx = *src_idx_ptr;

	src_z_ptr++;
	src_idx_ptr++;

	if (src_idx<0)
	  continue;
	
	float& dest_z = *(dest_z_ptr+ttable[c]);
	int& dest_idx = *(dest_idx_ptr+ttable[c]);
	
	if (dest_idx<0 || (dest_idx>=0 && dest_z>=src_z)) {
	  dest_idx = src_idx;
	  dest_z = src_z;
	}
      }
    }
    // set the points at 1e3 to 0 depth, for consistency
    for (int r = 0; r<dest_buffer.rows; r++) {
      float* z_ptr =dest_buffer.ptr<float>(r);
      for (int c = 0; c<dest_buffer.cols; c++) {
	if (*z_ptr>=big_value)
	  *z_ptr=0;
	z_ptr++;
      }
    }
  }

  void compareDepths(float& in_distance, 
		     int& in_num,
		     float& out_distance, 
		     int& out_num,
		     const FloatImage& depths1, const IntImage& indices1, 
		     const FloatImage& depths2, const IntImage& indices2, 
		     float dist, bool scale_z, FloatImage* result) {
    if (depths1.rows!=indices1.rows ||
	depths1.cols!=indices1.cols)
      throw std::runtime_error("compareDepths: image1 size mismatch");

    if (depths2.rows!=indices2.rows ||
	depths2.cols!=indices2.cols)
      throw std::runtime_error("compareDepths: image2 size mismatch");

    if (depths1.rows!=depths2.rows ||
	depths1.cols!=depths2.cols)
      throw std::runtime_error("compareDepths: image1 - image2 size mismatch");
    

    in_distance = 0;
    in_num = 0;
    out_distance = 0;
    out_num = 0;
    if (result) {
      result->create(depths1.rows, depths1.cols);
      *result = 6.0f;
    }
    for(int r = 0; r<depths1.rows; r++) {
      const float* d1_ptr = depths1.ptr<float>(r);
      const float* d2_ptr = depths2.ptr<float>(r);
      float* res_ptr = 0;
      if (result) {
	res_ptr=result->ptr<float>(r);
      }
      const int* i1_ptr = indices1.ptr<int>(r);
      const int* i2_ptr = indices2.ptr<int>(r);
      for (int c = 0; c<depths1.cols; c++){
	int i1 = *i1_ptr;
	float d1 = *d1_ptr;
	int i2 = *i2_ptr;
	float d2 = *d2_ptr;
	if (res_ptr)
	  *res_ptr = 6;

	if (i1>-1 && i2>-1) {
	  float avg = .5* (d1+d2);
	  float d = fabs(d1-d2);
	  d=scale_z ? d/avg : d;
	  if (d<dist){
	    if (res_ptr)
	      *res_ptr = 6;
	    in_num ++;
	    in_distance += d;
	  } else {
	    if (res_ptr && d<0.5)
	      *res_ptr = d;
	    out_num ++;
	    out_distance += d;
	  }
	}
	i1_ptr++;
	i2_ptr++;
	d1_ptr++;
	d2_ptr++;
	    if (res_ptr)
	res_ptr++;
      }
    }
  }

  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, float scale) {
    assert(src.type() != CV_32FC1 && "convert_32FC1_to_16UC1: source image of different type from 32FC1");
    const float* sptr = (const float*)src.data;
    int size = src.rows * src.cols;
    const float* send = sptr + size;
    dest.create(src.rows, src.cols, CV_16UC1);
    dest.setTo(cv::Scalar(0));
    unsigned short* dptr = (unsigned short*)dest.data;
    while(sptr < send) {
      if(*sptr >= 1e9f) { *dptr = 0; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
    }
  }

  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale) {
    assert(src.type() != CV_32FC1 && "convert_32FC1_to_16UC1: source image of different type from 32FC1");
    const int* mptr = (const int*)mask.data;
    const float* sptr = (const float*)src.data;
    int size = src.rows * src.cols;
    const float* send = sptr + size;
    dest.create(src.rows, src.cols, CV_16UC1);
    dest.setTo(cv::Scalar(0));
    unsigned short* dptr = (unsigned short*)dest.data;
    while(sptr < send) {
      if(*mptr < 0) { *dptr = 0; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
      ++mptr;
    }
  }

  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, float scale) {
    assert(src.type() != CV_16UC1 && "convert_16UC1_to_32FC1: source image of different type from 16UC1");
    const unsigned short* sptr = (const unsigned short*)src.data;
    int size = src.rows * src.cols;
    const unsigned short* send = sptr + size;
    dest.create(src.rows, src.cols, CV_32FC1);
    dest.setTo(cv::Scalar(0.0f));
    float* dptr = (float*)dest.data;
    while(sptr < send) {
      if(*sptr == 0) { *dptr = 1e9f; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
    }
  }  

  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale) {
    assert(src.type() != CV_16UC1 && "convert_16UC1_to_32FC1: source image of different type from 16UC1");
    const int* mptr = (const int*)mask.data;
    const unsigned short* sptr = (const unsigned short*)src.data;
    int size = src.rows * src.cols;
    const unsigned short* send = sptr + size;
    dest.create(src.rows, src.cols, CV_32FC1);
    dest.setTo(cv::Scalar(0.0f));
    float* dptr = (float*)dest.data;
    while(sptr < send) {
      if(*mptr < 0) { *dptr = 1e9f; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
      ++mptr;
    }
  }  

  void add_16UC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2) {
    assert(src1.type() != CV_16UC1 && src2.type() != CV_16UC1 && "add_16UC1: source images are of different type from 16UC1");
    assert(src1.cols != src2.cols && "add_16UC1: source images are of not of the same size");
    assert(src1.cols != src2.cols && "add_16UC1: source images are of not of the same size");
    
    const unsigned short* sptr1 = (const unsigned short*)src1.data;
    const unsigned short* sptr2 = (const unsigned short*)src2.data;
    int size = src1.rows * src1.cols;
    const unsigned short* send = sptr1 + size;
    dest.create(src1.rows, src1.cols, CV_16UC1);
    dest.setTo(cv::Scalar(0));
    unsigned short* dptr = (unsigned short*)dest.data;
    while(sptr1 < send) {
      *dptr = (*sptr1) + (*sptr2);
      ++dptr;
      ++sptr1;
      ++sptr2;
    }
  }
  
  void sub_16UC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2) {
    assert(src1.type() != CV_16UC1 && src2.type() != CV_16UC1 && "add_16UC1: source images are of different type from 16UC1");
    assert(src1.cols != src2.cols && "add_16UC1: source images are of not of the same size");
    assert(src1.cols != src2.cols && "add_16UC1: source images are of not of the same size");
    
    const unsigned short* sptr1 = (const unsigned short*)src1.data;
    const unsigned short* sptr2 = (const unsigned short*)src2.data;
    int size = src1.rows * src1.cols;
    const unsigned short* send = sptr1 + size;
    dest.create(src1.rows, src1.cols, CV_16UC1);
    dest.setTo(cv::Scalar(0));
    unsigned short* dptr = (unsigned short*)dest.data;
    while(sptr1 < send) {
      *dptr = (*sptr1) - (*sptr2);
      ++dptr;
      ++sptr1;
      ++sptr2;
    }
  }

  void add_32FC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2) {
    assert(src1.type() != CV_16UC1 && src2.type() != CV_16UC1 && "add_32FC1: source images are of different type from 32FC1");
    assert(src1.cols != src2.cols && "add_32FC1: source images are of not of the same size");
    assert(src1.cols != src2.cols && "add_32FC1: source images are of not of the same size");
    
    const float* sptr1 = (const float*)src1.data;
    const float* sptr2 = (const float*)src2.data;
    int size = src1.rows * src1.cols;
    const float* send = sptr1 + size;
    dest.create(src1.rows, src1.cols, CV_32FC1);
    dest.setTo(cv::Scalar(0));
    float* dptr = (float*)dest.data;
    while(sptr1 < send) {
      *dptr = (*sptr1) + (*sptr2);
      ++dptr;
      ++sptr1;
      ++sptr2;
    }
  }
  
  void sub_32FC1(cv::Mat& dest, const cv::Mat& src1, const cv::Mat& src2) {
        assert(src1.type() != CV_16UC1 && src2.type() != CV_16UC1 && "add_32FC1: source images are of different type from 32FC1");
    assert(src1.cols != src2.cols && "add_32FC1: source images are of not of the same size");
    assert(src1.cols != src2.cols && "add_32FC1: source images are of not of the same size");
    
    const float* sptr1 = (const float*)src1.data;
    const float* sptr2 = (const float*)src2.data;
    int size = src1.rows * src1.cols;
    const float* send = sptr1 + size;
    dest.create(src1.rows, src1.cols, CV_32FC1);
    dest.setTo(cv::Scalar(0));
    float* dptr = (float*)dest.data;
    while(sptr1 < send) {
      *dptr = (*sptr1) - (*sptr2);
      ++dptr;
      ++sptr1;
      ++sptr2;
    }
  }


    void computeSimpleNormals(Float3Image& normal_image,
			      const Float3Image& points_image,
			      int col_gap,
			      int row_gap,
			      float max_distance = 0.1) {

    normal_image.create(points_image.rows, points_image.cols);
    float squared_max_distance=max_distance*max_distance;
    for (int r = row_gap; r < points_image.rows - row_gap; r++) {
      const cv::Vec3f* up_row_ptr=points_image.ptr<const cv::Vec3f>(r-row_gap);
      const cv::Vec3f* row_ptr=points_image.ptr<const cv::Vec3f>(r);
      const cv::Vec3f* down_row_ptr=points_image.ptr<const cv::Vec3f>(r+row_gap);
      cv::Vec3f* dest_row_ptr=normal_image.ptr<cv::Vec3f>(r);
      for (int c = col_gap;
	   c < points_image.cols - col_gap; +
	     ++c, ++up_row_ptr, ++row_ptr, ++down_row_ptr, ++dest_row_ptr ) {
	dest_row_ptr[c]=cv::Vec3f(0.f, 0.f, 0.f);
	const cv::Vec3f& p = points_image.at<cv::Vec3f>(r,c);
	// if z is null, skip; 
	if (p[2]==0)
	  continue;
      
	const cv::Vec3f& px00 = row_ptr[c-col_gap];
	if (px00[2]==0)
	  continue;
      
	const cv::Vec3f& px01 = row_ptr[c+col_gap];
	if (px01[2]==0)
	  continue;

	const cv::Vec3f& py00 = up_row_ptr[c];
	if (py00[2]==0)
	  continue;

	const cv::Vec3f& py01 = down_row_ptr[c];
	if (py00[2]==0)
	  continue;

	cv::Vec3f dx = px01 - px00;
	cv::Vec3f dy = py01 - py00;
	if (dx.dot(dx) > squared_max_distance || 
	    dy.dot(dy) > squared_max_distance) { continue; }

	cv::Vec3f n = dy.cross(dx);
	n = normalize(n);
	if (n.dot(p) > 0) { n = -n; }
	dest_row_ptr[c] = n;
      }
    }
  }

  void normalBlur(Float3Image& dest, const Float3Image& src, int window, int start) {

    dest = Float3Image::zeros(src.size());
    int rows = dest.rows;
    int cols = dest.cols;
    Float3Image normal_integral;
    cv::integral(src, normal_integral, CV_32F);

    for (int r = start + window; r < rows - start - window; r++) {
      for (int c = start + window; c < cols - start - window; c++) {
	cv::Vec3f m11=normal_integral.at<cv::Vec3f>(r+window, c+window);
	cv::Vec3f m00=normal_integral.at<cv::Vec3f>(r-window, c-window);
	cv::Vec3f m01=normal_integral.at<cv::Vec3f>(r-window, c+window);
	cv::Vec3f m10=normal_integral.at<cv::Vec3f>(r+window, c-window);
	cv::Vec3f n_sum=m11+m00-m01-m10;
	if (n_sum.dot(n_sum)>0.2)
	  dest.at<cv::Vec3f>(r, c)= normalize(n_sum);
	else
	  dest.at<cv::Vec3f>(r, c) = cv::Vec3f(0,0,0);
      }
    }
  }

#ifdef _DEBUG_MERGE_

#define checkCumVal(x,msg)						\
  if (x.cumulativeValues[x##Idx]<=0){					\
    cerr << "idx: " << x##Idx << endl;					\
      cerr << "p:" << x.pointAccumulators[x##Idx].transpose() << endl;	\
      cerr << "n:" << x.normalAccumulators[x##Idx].transpose() << endl; \
      cerr << "c:" << x.cumulativeValues[x##Idx]<< endl;		\
      throw std::runtime_error(msg);					\
  }
#else
#define checkCumVal(x,msg)
#endif

  void merge(FloatImage& destBuffer, IndexImage& destIndices, Cloud3D& dest,
	     FloatImage& srcBuffer, IndexImage& srcIndices, Cloud3D& src,
	     float normalThreshold,
	     float distanceThreshold) {

#ifdef _DEBUG_MERGE_
    cerr << "Merge: " << endl;
    cerr << "destSize: " << dest.size() << " srcSize: " << src.size() << endl;
#endif //_DEBUG_MERGE_
    
    normalThreshold = cos(normalThreshold);
    int newPoints = 0;
    for (int c = 0; c < destBuffer.cols; c++) {
      for (int r = 0; r < destBuffer.rows; r++) {
	int& srcIdx = srcIndices.at<int>(r, c);
	int& destIdx = destIndices.at<int>(r, c);

	// add a point if it appars only in the src and are undefined in the dest
	if (srcIdx < 0 || destIdx < 0) {
	  if (srcIdx > -1 && destIdx < 0)
	    newPoints ++;
	  continue;
	}

	checkCumVal(dest, "before dest");
	checkCumVal(src, "before src");

	float destDepth = destBuffer.at<float>(r, c);
	float srcDepth = srcBuffer.at<float>(r, c);
	float davg = 0.5 * (destDepth + srcDepth);
	// if a new point appears a lot in front an old one add it
	float scaledDistance = (destDepth - srcDepth) / davg;

	if ( scaledDistance > distanceThreshold) {
	  destIdx = -1;
	  newPoints++;
	  continue;
	}

	// if a new point appears a lot behind an old replace the old
	if ( scaledDistance < -distanceThreshold) {
	  dest[destIdx] = src[srcIdx];
	  srcIdx = -1;
	  destIdx = -1;
	  continue;
	}

	// if the normals do not match do nothing
	Eigen::Vector3f destNormal = dest[destIdx].normal();
	if (!dest[destIdx].isNormalized())
	  destNormal /= dest[destIdx].accumulator();

	Eigen::Vector3f srcNormal = src[srcIdx].normal();
	if (!src[srcIdx].isNormalized())
	  srcNormal /= src[srcIdx].accumulator();
	if (destNormal.dot(srcNormal) < normalThreshold) {
	  destIdx = -1;
	  //newPoints++;
	  srcIdx = -1;
	  continue;
	}

	// merge the points
	dest[destIdx] += src[srcIdx];
	srcIdx = -1;

	checkCumVal(dest, "merge in bounds");
      }
    }

#ifdef _DEBUG_MERGE_
    cerr << "dest expected final size: " << dest.size() + newPoints << endl;
#endif //_DEBUG_MERGE_
    
    // recompute all the touched points
    for (int c = 0; c < destBuffer.cols; c++) {
      for (int r = 0; r < destBuffer.rows; r++) {
	int& destIdx = destIndices.at<int>(r, c);
	if (destIdx < 0)
	  continue;
	dest[destIdx].normalize();
      }
    }

    int k = dest.size();
    dest.resize(dest.size() + newPoints);
    for (int c = 0; c < srcBuffer.cols; c++) {
      for (int r = 0; r < srcBuffer.rows; r++) {
	int& srcIdx = srcIndices.at<int>(r, c);
	if (srcIdx < 0)
	  continue;
	dest[k] = src[srcIdx];
	k++;
      }
    }
#ifdef _DEBUG_MERGE_
    cerr << "expected size: " << dest.size() << " newPoints: " << newPoints << " finalSize: " << k << endl;
#endif //_DEBUG_MERGE_
  }

}
