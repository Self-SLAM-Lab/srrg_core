#include "base_sensor_message.h"
#include "srrg_types/defs.h"
#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>

namespace srrg_core {
  
  using namespace std;
  using namespace srrg_core;
  
  BaseSensorMessage::BaseSensorMessage(const std::string& topic_, const std::string& frame_id, int seq_, double timestamp_):
    _topic (topic_), _frame_id(frame_id), _seq(seq_), _timestamp(timestamp_){
    _has_odom = false;
    _has_imu = false;
    _offset.setIdentity();
    _imu.setIdentity();
    _odometry.setIdentity();
  }
  
  void BaseSensorMessage::fromStream(std::istream& is) {
    is >> _topic;
    is >> _frame_id;
    is >> _seq;
    is >> _timestamp;
    _has_odom=false;
    _has_imu=false;
    _offset.setIdentity();
    _imu.setIdentity();
    _odometry.setIdentity();
    BaseMessage::fromStream(is);
    Vector6f v;
    for(int i=0;i<v.rows(); i++)      
      is >> v(i);
    _offset = v2t(v);

    is >> _has_odom;
    if (_has_odom) {
      for(int i=0;i<v.rows(); i++)      
	is >> v(i);
      _odometry = v2t(v);
    }
    is >> _has_imu;
    if (_has_imu) {
      for(int i=0;i<v.rows(); i++)      
	is >> v(i);
      _imu = v2t(v);
    }
  }
  
  void  BaseSensorMessage::toStream(std::ostream& os) const {
    BaseMessage::toStream(os);
    char buf[1024];
    sprintf(buf, "%s %s %08d %.5lf", _topic.c_str(), _frame_id.c_str(), _seq, _timestamp);
    os << buf;
    os << " ";
    Vector6f v=t2v(_offset);
    os << v.transpose() << " ";
    os << _has_odom << " ";
    if (_has_odom) {
      v=t2v(_odometry);
      os << v.transpose() << " ";
    }
    os << _has_imu << " ";
    if (_has_imu) {
      v=t2v(_imu);
      os << v.transpose() << " ";
    }
  }


      
  void BaseSensorMessage::serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    data.setString("topic", _topic);
    data.setString("frame_id", _frame_id);
    data.setInt("seq", _seq);
    data.setDouble("timestamp", _timestamp);
    t2v(_offset).toBOSS(data, "offset");
    if (_has_odom)
      t2v(_odometry).toBOSS(data, "odometry");
    if (_has_imu)
      t2v(_imu).toBOSS(data, "imu");
  }

  void BaseSensorMessage::deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    _topic = data.getString("topic");
    _frame_id=data.getString("frame_id");
    _seq = data.getInt("seq");
    _timestamp=data.getDouble("timestamp");
    Vector6f offset_v;
    offset_v.fromBOSS(data, "offset");
    _offset=v2t(offset_v);
    if (data.getField("odometry")){
      _has_odom=true;
      Vector6f odom_v;
      odom_v.fromBOSS(data, "odometry");
      _odometry=v2t(odom_v);
    }
    if (data.getField("imu")){
      _has_imu=true;
      Vector6f imu_v;
      imu_v.fromBOSS(data, "imu");
      _imu=v2t(imu_v);
    }
  }

  
}
