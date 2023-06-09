#pragma once

#include <iostream>

#include <srrg_types/defs.h>

#include <srrg_boss/blob.h>

namespace srrg_core {

struct RichPoint3D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  inline RichPoint3D(const Eigen::Vector3f& p = Eigen::Vector3f::Zero(),
                   const Eigen::Vector3f& n = Eigen::Vector3f::Zero(),
                   float a = 0.0f,
                   const Eigen::Vector3f& c = Eigen::Vector3f::Zero()) {
    _point = p;
    _normal = n;
    _rgb = c;
    _accumulator = a;
    _is_normalized = true;
  }

  inline RichPoint3D& operator+=(const RichPoint3D& rp) {
    denormalize();
    RichPoint3D rp2(rp);
    rp2.denormalize();
    _point += rp2._point;
    _normal += rp2._normal;
    _rgb += rp2._rgb;
    _accumulator += rp2._accumulator;
    return *this;
  }

  inline bool isNormalized() const { return _is_normalized; }
  inline const Eigen::Vector3f& point() const { return _point; }
  inline const Eigen::Vector3f& normal() const { return _normal; }
  inline const Eigen::Vector3f& rgb() const { return _rgb; }

  inline float accumulator() const { return _accumulator; }
  inline void transformInPlace(Eigen::Isometry3f iso) {
    _point = iso * _point;
    _normal = iso.linear() * _normal;
  }
  inline RichPoint3D transform(const Eigen::Isometry3f& iso) const {
    return RichPoint3D(iso * _point, iso.linear() * _normal, _accumulator, _rgb);
  }

  inline void denormalize() {
    if (!_is_normalized) {
      return;
    }
    _point *= _accumulator;
    _normal *= _accumulator;
    _rgb *= _accumulator;
    _is_normalized = false;
  }

  inline void normalize() {
    if (_is_normalized) {
      return;
    }
    if (_accumulator > 0) {
      float iv = 1. / _accumulator;
      _point *= iv;
      _normal *= iv;
      _rgb *= iv;
      if(_normal.squaredNorm()>0)
	_normal.normalize();
    } else {
      _point.setZero();
      _normal.setZero();
      _rgb.setZero();
    }
    _is_normalized = true;
  }

  Eigen::Vector3f _point;
  Eigen::Vector3f _normal;
  Eigen::Vector3f _rgb;
  float _accumulator;
  bool _is_normalized;
};

typedef std::vector<RichPoint3D, Eigen::aligned_allocator<RichPoint3D> >
    RichPoint3DVector;

/**)
   This class represents a 3D model, as a collection of rich points
 */

struct Cloud3D : public srrg_boss::BLOB, public RichPoint3DVector {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  //! applies the transformation to each entity in the model, doing side effect.
  //! @param dest: output
  //! @param T: the transform

  void transformInPlace(const Eigen::Isometry3f& T);
  //! applies the transformation to each entity in the model, but writes the
  //output in dest.
  //! @param dest: output
  //! @param T: the transform
  void transform(Cloud3D& dest, const Eigen::Isometry3f& T) const;

  //! adds other to this point cloud, doing side effect
  void add(const Cloud3D& other);

  //! draws a cloud if invoked in a gl context
  virtual void draw(int name = -1) const;

  //! saves the cloud in a binary stream, optimized for speed
  virtual void write(std::ostream& os) const;

  //! loads the cloud from a binary stream, optimized for speed
  virtual bool read(std::istream& is);

  //! clips to a maxRange around a pose
  void clip(float range,
            const Eigen::Isometry3f& pose = Eigen::Isometry3f::Identity());

  //! computes the bounding box of a cloud.
  //! p1: lower x, lower y, lower z
  void computeBoundingBox(Eigen::Vector3f& lower,
                          Eigen::Vector3f& higher) const;


  //! prunes the points in model, computing a scaled average
  //! one point will survive for each voxel of side res
  void voxelize(float res);

};

typedef srrg_boss::BLOBReference<Cloud3D> Cloud3DBLOBReference;


}
