#ifndef POSE_MESSAGE_H
#define POSE_MESSAGE_H

#include "base_sensor_message.h"
#include <string>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <srrg_types/types.hpp>

namespace srrg_core
{

class PoseMessage: public BaseSensorMessage
{

//ds ctor/dtor (MUST NOT THROW)
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseMessage(const std::string& topic_ = "",
                const std::string& frame_id_ = "",
                const int32_t& sequence_number_ = -1,
                const double& timestamp_seconds_ = -1.0);

//ds accessors
public:

  virtual void serialize(srrg_boss::ObjectData& data_, srrg_boss::IdContext& context);
  virtual void deserialize(srrg_boss::ObjectData& data_, srrg_boss::IdContext& context_);

  //ds overrides
  virtual void fromStream(std::istream& is_message_);
  virtual void toStream(std::ostream& os_message_) const;
  virtual const std::string& tag() const {return _tag;}

  //ds setters
  inline void setPose(const Eigen::Isometry3d& pose_) {_pose = pose_;}
  inline void setCovariance(const Matrix6d& covariance_) {_covariance = covariance_;}

  //ds getters
  inline const Eigen::Isometry3d pose() const {return _pose;}
  inline const Matrix6d covariance() const {return _covariance;}

//ds members (inheritable)
protected:

  //ds pose specific
  Eigen::Isometry3d _pose;
  Matrix6d _covariance;

//ds tag
private:

  static const std::string _tag;
};
}  //namespace srrg_core

#endif //#define POSE_MESSAGE_H
