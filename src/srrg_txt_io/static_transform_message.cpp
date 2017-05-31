#include "static_transform_message.h"
#include "srrg_types/defs.h"
#include <cstdio>
#include <cstring>
#include "message_factory.h"
#include <stdexcept>

namespace srrg_core {

  using namespace std;
  using namespace Eigen;
  using namespace srrg_core;
  
  StaticTransformMessage::StaticTransformMessage(const std::string& from_frame_id, 
		     const std::string& to_frame_id, 
		     const Eigen::Isometry3f& transform_):
      _from_frame_id(from_frame_id),
      _to_frame_id(to_frame_id),
      _transform(transform_),
      _parent(0){
    }

  void StaticTransformMessage::serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    data.setString("from_frame_id", _from_frame_id);
    data.setString("to_frame_id", _to_frame_id);
    t2v(_transform).toBOSS(data, "transform");
  }

  void StaticTransformMessage::deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    _from_frame_id=data.getString("from_frame_id");
    _to_frame_id=data.getString("to_frame_id");
    Vector6f transform_v;
    transform_v.fromBOSS(data, "transform");
    _transform=v2t(transform_v);
  }

  void StaticTransformMessage::fromStream(std::istream& is) {
    is >> _from_frame_id;
    is >> _to_frame_id;
    std::string fmt;
    is >> fmt;
    if (fmt == "tq") {
      cerr << "format tq" << endl;
      Vector6f t;
      for (int i=0; i<t.rows(); i++)
	is >> t(i);
      _transform = v2t(t);
    } else if (fmt == "trpy") {
      cerr << "format trpy" << endl;
      Vector6f t;
      for (int i=0; i<t.rows(); i++)
	is >> t(i);
      _transform.translation()=t.block<3,1>(0,0);
      _transform.linear() = (
	AngleAxisf(t(3), Vector3f::UnitX())
	* AngleAxisf(t(4), Vector3f::UnitY())
	* AngleAxisf(t(5)*M_PI, Vector3f::UnitZ())).toRotationMatrix();
    } else if (fmt == "Rt") {
      cerr << "format Rt" << endl;
      for (int i=0; i<3; i++) 
	for (int j=0; j<4; j++)
	  is >> _transform.matrix()(i,j);
      _transform.matrix().row(3) << 0,0,0,1;
    } else {
      throw std::runtime_error("unknown format");
    }

  }

  void  StaticTransformMessage::toStream(std::ostream& os) const {
    os << _from_frame_id << " " << _to_frame_id << " tq ";
    Vector6f v=t2v(_transform);
    os << v.transpose();
  }
 
  const std::string StaticTransformMessage::_tag="STATIC_TRANSFORM";

  const std::string& StaticTransformMessage::tag() const { return _tag; }

  static MessageFactory::MessageRegisterer<StaticTransformMessage> registerer;



}
