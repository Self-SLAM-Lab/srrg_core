#pragma once

#include <iostream>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>

namespace srrg_core {

    //!a vector of Vector2f with alignment
  typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;

  //!a vector of Vector3f with alignment
  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;

  //!a vector of Vector2f with alignment
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

  //!a vector of Matrix3f with alignment
  typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > Matrix3fVector;

  //!a vector of Matrix2f with alignment
  typedef std::vector<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f> > Matrix2fVector;

  //!a 4x3 float matrix
  typedef Eigen::Matrix<float, 4, 3> Matrix4_3f;

  //!a 4x6 float matrix
  typedef Eigen::Matrix<float, 4, 6> Matrix4_6f;

  //!a 2x6 float matrix
  typedef Eigen::Matrix<float, 2, 6> Matrix2_6f;

  //!a 3x6 float matrix
  typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

  //!a 3x6 double matrix
  typedef Eigen::Matrix<double, 3, 6> Matrix3_6d;

  //!a 5x5 float matrix
  typedef Eigen::Matrix<float, 5, 5> Matrix5f;

  //!a 6x6 float matrix
  typedef Eigen::Matrix<float, 6, 6> Matrix6f;

  //!a 6x6 double matrix
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  //!a 6   float vector
  typedef Eigen::Matrix<float, 6, 1> Vector6f;
  
  //!a 6 double vector
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  //!a 7   float vector
  typedef Eigen::Matrix<float, 7, 1> Vector7f;

  //!a 9x6 float matrix 
  typedef Eigen::Matrix<float, 9, 6> Matrix9_6f;

  //!a 9x9 float matrix 
  typedef Eigen::Matrix<float, 9, 9> Matrix9f;

  //!a 9 float vector
  typedef Eigen::Matrix<float, 9, 1> Vector9f;

  //!a 6x3 float matrix
  typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;

  //!a 6x3 double matrix
  typedef Eigen::Matrix<double, 6, 3> Matrix6_3d;

  
  //!check if an Eigen type contains a nan element
  //!@returns true if at least one element of
  //!the argument is null
  template <class T> 
  bool isNan(const T& m){
    for (int i=0; i< m.rows(); i++) {
      for (int j=0; j< m.cols(); j++) {
	float v = m(i,j);
	if ( std::isnan( v ) )
	  return true;
      }
    }
    return false;
  }

  //!converts from 6 vector to isometry
  //!@param t: a vector (tx, ty, tz, qx, qy, qz) reptesenting the transform.
  //!(qx, qy, qz) are the imaginary part of a normalized queternion, with qw>0.
  //!@returns the isometry corresponding to the transform described by t
  inline Eigen::Isometry3f v2t(const Vector6f& t){
    Eigen::Isometry3f T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
      w=sqrt(1-w);
      T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
      Eigen::Vector3f q=t.block<3,1>(3,0);
      q.normalize();
      T.linear()=Eigen::Quaternionf(0, q(0), q(1), q(2)).toRotationMatrix();
    }
    return T;
  }
  inline Eigen::Isometry3d v2t(const Eigen::Matrix<double, 6, 1>& t){
    Eigen::Isometry3d T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
      w=sqrt(1-w);
      T.linear()=Eigen::Quaterniond(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
      Eigen::Vector3d q=t.block<3,1>(3,0);
      q.normalize();
      T.linear()=Eigen::Quaterniond(0, q(0), q(1), q(2)).toRotationMatrix();
    }
    return T;
  }

  //!converts from isometry to 6 vector                                                                   
  //!@param t: an isometry
  //!@returns a vector (tx, ty, tz, qx, qy, qz) reptesenting the transform.
  //!(qx, qy, qz) are the imaginary part of a normalized queternion, with qw>0.
  inline Vector6f t2v(const Eigen::Isometry3f& t){
    Vector6f v;
    v.head<3>()=t.translation();
    Eigen::Quaternionf q(t.linear());
    v(3) = q.x();
    v(4) = q.y();
    v(5) = q.z();
    if (q.w()<0)
      v.block<3,1>(3,0) *= -1.0f;
    return v;
  }

  //!converts from isometry to 7 vector                                                                   
  //!@param t: an isometry
  //!@returns a vector (tx, ty, tz, qx, qy, qz, qw) reptesenting the transform.
  //!be aware! the output quaternion is not normalized
  inline Vector7f t2w(const Eigen::Isometry3f& t){
    Vector7f v;
    v.head<3>()=t.translation();
    Eigen::Quaternionf q(t.linear());
    v(3) = q.x();
    v(4) = q.y();
    v(5) = q.z();
    v(6) = q.w();
    return v;
  }
  
  //!converts from isometry2f to isometry3f                                                                   
  //!@param t: an isometry2f
  //!@returns an isometry3f
  inline Eigen::Isometry3f toIsometry3f(const Eigen::Isometry2f& isometry2f){
    Eigen::Isometry3f isometry3f;
    isometry3f.linear().block<2,2>(0,0) = isometry2f.linear();
    isometry3f.translation().head<2>() = isometry2f.translation();
    return isometry3f;
  }

  //!converts from isometry3f to isometry2f                                                                   
  //!@param t: an isometry3f
  //!@returns an isometry2f
  inline Eigen::Isometry2f toIsometry2f(const Eigen::Isometry3f& isometry3f){
    Eigen::Isometry2f isometry2f;
    isometry2f.linear() = isometry3f.linear().block<2,2>(0,0);
    isometry2f.translation() = isometry3f.translation().head<2>();
    return isometry2f;
  }


  template<typename Derived1>
  inline Eigen::Transform<Derived1, 2, Eigen::Isometry> iso2(const Eigen::Transform<Derived1, 3, Eigen::Isometry> &B){
    Eigen::Transform<Derived1, 2, Eigen::Isometry> A;
    A.template translation() = B.template translation().head(2);
    Eigen::Matrix<Derived1,3,1> v = B.template linear().eulerAngles(0,1,2);
    Eigen::Rotation2D<Derived1> rot(v(2));
    Eigen::Matrix<Derived1,2,2> rot_e(rot);
    A.template linear() = rot_e;

    return A;
  }
  
  //!computes the cross product matrix of the vector argument
  //!@param p: the vector
  //!@returns a 3x3 matrix 
  inline Eigen::Matrix3f skew(const Eigen::Vector3f& p){
    Eigen::Matrix3f s;
    s << 
      0,  -p.z(), p.y(),
      p.z(), 0,  -p.x(), 
      -p.y(), p.x(), 0;
    return s;
  }
  inline Eigen::Matrix3d skew(const Eigen::Vector3d& p){
    Eigen::Matrix3d s;
    s <<
      0,  -p.z(), p.y(),
      p.z(), 0,  -p.x(),
      -p.y(), p.x(), 0;
    return s;
  }

  inline Eigen::Isometry2f v2t(const Eigen::Vector3f& t){
    Eigen::Isometry2f T;
    T.setIdentity();
    T.translation()=t.head<2>();
    float c = cos(t(2));
    float s = sin(t(2));
    T.linear() << c, -s, s, c;
    return T;
  }

  inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
    Eigen::Vector3f v;
    v.head<2>()=t.translation();
    v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
    return v;
  }


  inline Eigen::Matrix3f Rx(float rot_x){
    float c=cos(rot_x);
    float s=sin(rot_x);
    Eigen::Matrix3f R;
    R << 1,  0, 0,
      0,  c,  -s,
      0,  s,  c;
    return R;
  }
  inline Eigen::Matrix3d Rx(const double& rot_x){
    const double c=cos(rot_x);
    const double s=sin(rot_x);
    Eigen::Matrix3d R;
    R << 1,  0, 0,
      0,  c,  -s,
      0,  s,  c;
    return R;
  }
  
  inline Eigen::Matrix3f Ry(float rot_y){
    float c=cos(rot_y);
    float s=sin(rot_y);
    Eigen::Matrix3f R;
    R << c,  0,  s,
      0 , 1,  0,
      -s,  0, c;
    return R;
  }
  inline Eigen::Matrix3d Ry(const double& rot_y){
    const double c=cos(rot_y);
    const double s=sin(rot_y);
    Eigen::Matrix3d R;
    R << c,  0,  s,
      0 , 1,  0,
      -s,  0, c;
    return R;
  }

  inline Eigen::Matrix3f Rz(float rot_z){
    float c=cos(rot_z);
    float s=sin(rot_z);
    Eigen::Matrix3f R;
    R << c,  -s,  0,
      s,  c,  0,
      0,  0,  1;
    return R;
  }
  inline Eigen::Matrix3d Rz(const double& rot_z){
    const double c=cos(rot_z);
    const double s=sin(rot_z);
    Eigen::Matrix3d R;
    R << c,  -s,  0,
      s,  c,  0,
      0,  0,  1;
    return R;
  }

  
  inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
    Eigen::Isometry3f T;
    T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
    T.translation()=v.head<3>();
    return T;
  }
  inline Eigen::Isometry3d v2tEuler(const Eigen::Matrix<double, 6, 1>& v){
    Eigen::Isometry3d T;
    T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
    T.translation()=v.head<3>();
    return T;
  }


  inline Eigen::Matrix2f skew(const Eigen::Vector2f& p){
    Eigen::Matrix2f s;
    s << 
      0,  -p.y(),
      p.x(), 0;
    return s;
  }

  inline void normalizeAngle(float &theta){
    theta = atan2(sin(theta),cos(theta));
  }
  inline void normalizeAngle(double &theta){
    theta = atan2(sin(theta),cos(theta));
  }  

  typedef std::vector<int> IntVector;  

  typedef std::vector<float> FloatVector;

  typedef std::vector<std::pair<int, int> > IntPairVector;


}
