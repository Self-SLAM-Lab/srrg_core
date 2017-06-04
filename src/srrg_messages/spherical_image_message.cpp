#include "spherical_image_message.h"
#include <cstdio>
#include <cstring>
#include "message_factory.h"

namespace srrg_core {
  using namespace std;

  SphericalImageMessage::SphericalImageMessage(const std::string& topic_, const std::string& frame_id, int seq_, double timestamp_):
    BaseImageMessage(topic_,frame_id, seq_,timestamp_){
    _camera_matrix.setZero();
  }

  
  // BOSS interface
  void SphericalImageMessage::serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    BaseImageMessage::serialize(data,context);
    _camera_matrix.toBOSS(data, "camera_matrix");
  }

  void SphericalImageMessage::deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    BaseImageMessage::deserialize(data,context);
    _camera_matrix.fromBOSS(data, "camera_matrix");
  }

  const std::string SphericalImageMessage::_tag="SPHERICAL_IMAGE_MESSAGE";

  const std::string& SphericalImageMessage::tag() const { return _tag; }

  void SphericalImageMessage::fromStream(std::istream& is) {
    BaseImageMessage::fromStream(is);
    for (int r =0; r<4; r++)
      is >> _camera_matrix(r);
    _is_fetched = false;
  }

  void SphericalImageMessage::toStream(std::ostream& os) const {
    BaseImageMessage::toStream(os);
    os << " ";
    for (int r =0; r<4; r++)
	os << _camera_matrix(r) << " "; 
  }

  static MessageFactory::MessageRegisterer<SphericalImageMessage> registerer;

  BOSS_REGISTER_CLASS(SphericalImageMessage);

}
