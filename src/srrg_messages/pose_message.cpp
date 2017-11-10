#include "pose_message.h"

#include <stdio.h>
#include "message_factory.h"

namespace srrg_core
{

const std::string PoseMessage::_tag = "POSE_MESSAGE";

PoseMessage::PoseMessage(const std::string& topic_,
                         const std::string& frame_id_,
                         const int32_t& sequence_number_,
                         const double& timestamp_seconds_) {}

void PoseMessage::serialize(srrg_boss::ObjectData& data_, srrg_boss::IdContext& context_) {
  BaseSensorMessage::serialize(data_,context_);
  SRRG_TO_BOSS_MATRIX(data_, pose)
  SRRG_TO_BOSS(data_, covariance)
}

void PoseMessage::deserialize(srrg_boss::ObjectData& data_, srrg_boss::IdContext& context_) {
  BaseSensorMessage::deserialize(data_,context_);
  SRRG_FROM_BOSS_MATRIX(data_, pose)
  SRRG_FROM_BOSS(data_, covariance)
}

void PoseMessage::fromStream(std::istream& is_message_) {

  //ds deprecated
  throw std::runtime_error("PoseMessage::fromStream is not supported");
}

void PoseMessage::toStream(std::ostream& os_message_) const {

  //ds deprecated
  throw std::runtime_error("PoseMessage::toStream is not supported");
}

  //ds register the message for reading
  static MessageFactory::MessageRegisterer<PoseMessage> registerer;
  BOSS_REGISTER_CLASS(PoseMessage);
} //namespace srrg_core
