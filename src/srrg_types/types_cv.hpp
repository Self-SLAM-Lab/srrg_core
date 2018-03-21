#pragma once

#include <iostream>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

#include <opencv2/opencv.hpp>


namespace srrg_core {

  //!a 5 float cv vector
  typedef cv::Vec<float, 5> Vec5f;
  
  //!a 7 float cv vector
  typedef cv::Vec<float, 7> Vec7f;
  

  /** \typedef UnsignedCharImage
   * \brief An unsigned char cv::Mat.
   */
  typedef cv::Mat_<unsigned char> UnsignedCharImage;
  
  /** \typedef CharImage
   * \brief A char cv::Mat.
   */
  typedef cv::Mat_<char> CharImage;

  /** \typedef UnsignedShortImage
   * \brief An unsigned short cv::Mat.
   */
  typedef cv::Mat_<unsigned short> UnsignedShortImage;
  
  /** \typedef UnsignedIntImage
   * \brief An unsigned int cv::Mat.
   */
  typedef cv::Mat_<unsigned int> UnsignedIntImage;
  
  /** \typedef IntImage
   * \brief An int cv::Mat.
   */
  typedef cv::Mat_<int> IntImage;

  /** \typedef Int4Image
   * \brief A 4D int cv::Mat.
   */
  typedef cv::Mat_<cv::Vec4i> Int4Image;

  /** \typedef IntervalImage
   * \brief A 4D int cv::Mat used to store intervals.
   */
  typedef Int4Image IntervalImage;

  /** \typedef FloatImage
   * \brief A float cv::Mat.
   */
  typedef cv::Mat_<float> FloatImage;

  /** \typedef Float3Image
   * \brief A 3D float cv::Mat.
   */
  typedef cv::Mat_<cv::Vec3f> Float3Image;

  /** \typedef Float5Image
   * \brief A 5D float cv::Mat.
   */
  typedef cv::Mat_<Vec5f> Float5Image;

  /** \typedef Float5Image
   * \brief A 7D float cv::Mat.
   */
  typedef cv::Mat_<Vec7f> Float7Image;
  
  /** \typedef DoubleImage
   * \brief A double cv::Mat.
   */
  typedef cv::Mat_<double> DoubleImage;
  
  /** \typedef RawDepthImage
   * \brief An unsigned char cv::Mat used to for depth images with depth values expressed in millimeters.
   */
  typedef UnsignedShortImage RawDepthImage;
  
  /** \typedef IndexImage
   * \brief An int cv::Mat used to save the indeces of the points of a depth image inside a vector of points.
   */
  typedef IntImage IndexImage;
  
  /** \typedef DepthImage
   * \brief A float cv::Mat used to for depth images with depth values expressed in meters.
   */
  typedef cv::Mat_<cv::Vec3b> RGBImage;

  /** used to represent rgb values
   */
  typedef std::vector<cv::Vec3b> RGBVector;

  
  //ds overloaded opencv/eigen converters: double
  inline cv::Mat_<double> toCv(const Eigen::Matrix<double, 3, 3>& matrix_eigen_) {
    cv::Mat_<double> matrix_opencv(3, 3);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 3; ++v) {
        matrix_opencv.at<double>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<double> toCv(const Eigen::Matrix<double, 3, 4>& matrix_eigen_) {
    cv::Mat_<double> matrix_opencv(3, 4);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 4; ++v) {
        matrix_opencv.at<double>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<double> toCv(const Eigen::Matrix<double, 5, 1>& vector_eigen_) {
    cv::Mat_<double> vector_opencv(5,1);
    for(uint32_t u = 0; u < 5; ++u) {
      vector_opencv.at<double>(u) = vector_eigen_(u);
    }
    return vector_opencv;
  }
  inline Eigen::Matrix<double, 3, 1> fromCv(const cv::Vec<double, 3>& vector_opencv_) {
    Eigen::Matrix<double, 3, 1> vector_eigen;
    for(uint32_t u = 0; u < 3; ++u) {
      vector_eigen(u) = vector_opencv_(u);
    }
    return vector_eigen;
  }

  //ds overloaded opencv/eigen converters: float
  inline cv::Mat_<float> toCv(const Eigen::Matrix<float, 3, 3>& matrix_eigen_) {
    cv::Mat_<float> matrix_opencv(3, 3);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 3; ++v) {
        matrix_opencv.at<float>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<float> toCv(const Eigen::Matrix<float, 3, 4>& matrix_eigen_) {
    cv::Mat_<float> matrix_opencv(3, 4);
    for(uint32_t u = 0; u < 3; ++u) {
      for(uint32_t v = 0; v < 4; ++v) {
        matrix_opencv.at<float>(u, v) = matrix_eigen_(u, v);
      }
    }
    return matrix_opencv;
  }
  inline cv::Mat_<float> toCv(const Eigen::Matrix<float, 5, 1>& vector_eigen_) {
    cv::Mat_<float> vector_opencv(5,1);
    for(uint32_t u = 0; u < 5; ++u) {
      vector_opencv.at<float>(u) = vector_eigen_(u);
    }
    return vector_opencv;
  }
  inline Eigen::Matrix<float, 3, 1> fromCv(const cv::Vec<float, 3>& vector_opencv_) {
    Eigen::Matrix<float, 3, 1> vector_eigen;
    for(uint32_t u = 0; u < 3; ++u) {
      vector_eigen(u) = vector_opencv_(u);
    }
    return vector_eigen;
  }

  //ml std vector to Eigen converters
  inline Eigen::Matrix<float, 3, 1> fromFloatVector3f(const FloatVector& float_vector_){
    Eigen::Matrix<float, 3, 1> vector_eigen;
    assert(float_vector_.size() == 3);
    for (size_t i=0; i<3; i++)
      vector_eigen[i] = float_vector_[i];
    
    return vector_eigen;
  }
  inline FloatVector toFloatVector3f(const Eigen::Matrix<float, 3, 1>& vector_eigen_){
    FloatVector float_vector;
    float_vector.resize(3);
    for (size_t i=0; i<3; i++)
      float_vector[i] = vector_eigen_[i];
    return float_vector;
  }
}
