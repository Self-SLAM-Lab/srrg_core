#include "laser_message.h"
#include "message_factory.h"
#include <iostream>

namespace srrg_core{
  using namespace std;

  LaserMessage::LaserMessage(const std::string& topic, const std::string& frame_id,int seq, double timestamp):
    BaseSensorMessage(topic, frame_id, seq, timestamp){
      
  }

  void LaserMessage::serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    BaseSensorMessage::deserialize(data, context);
    data.setFloat("min_range", _min_range);
    data.setFloat("max_range", _max_range);
    data.setFloat("angle_increment", _angle_increment);
    data.setFloat("time_increment", _time_increment);
    data.setFloat("scan_time", _scan_time);
    srrg_boss::ArrayData* ranges_array=new srrg_boss::ArrayData;
    for (size_t i=0; i<_ranges.size(); ++i)
      ranges_array->push_back(new srrg_boss::NumberData((float)_ranges[i]));
    data.setField("ranges", ranges_array);
    srrg_boss::ArrayData* intensities_array=new srrg_boss::ArrayData;
    for (size_t i=0; i<_intensities.size(); ++i)
      intensities_array->push_back(new srrg_boss::NumberData((float)_intensities[i]));
    data.setField("intensities", intensities_array);
  }

  void LaserMessage::deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context){
    BaseSensorMessage::serialize(data, context);
    _min_range=data.getFloat("min_range");
    _max_range=data.getFloat("max_range");
    _angle_increment=data.getFloat("angle_increment");
    _time_increment=data.getFloat("time_increment");
    _scan_time=data.getFloat("scan_time");

    srrg_boss::ArrayData& ranges_array= data.getField("ranges")->getArray();
    _ranges.clear();
    for (size_t i=0; i<ranges_array.size(); ++i)
      _ranges.push_back(ranges_array[i]);

     srrg_boss::ArrayData& intensities_array= data.getField("intensities")->getArray();
    _intensities.clear();
    for (size_t i=0; i<intensities_array.size(); ++i)
      _intensities.push_back(intensities_array[i]);
 }

  
  const std::string LaserMessage::_tag="LASER_MESSAGE";

  const std::string& LaserMessage::tag() const {
    return _tag;
  }
  void LaserMessage::fromStream(std::istream& is) {
    BaseSensorMessage::fromStream(is);
    is >> _min_range >> _max_range >> _min_angle >> _max_angle >> _angle_increment >> _time_increment >> _scan_time;
    int num_ranges;
    is >> num_ranges;
    if(num_ranges)
      _ranges.resize(num_ranges);
    else
      _ranges.clear();

    for (size_t i=0; i<num_ranges; i++){
      is >> _ranges[i];
    }
      
    int num_intensities;
    is >> num_intensities;
    if(num_intensities)
      _intensities.resize(num_intensities);
    else
      _intensities.clear();
      
    for (size_t i=0; i<num_intensities; i++){
      is >> _intensities[i];
    }
      
  }

  void  LaserMessage::toStream(std::ostream& os) const {
    BaseSensorMessage::toStream(os);
    os << " " << _min_range
       << " " << _max_range
       << " " << _min_angle
       << " " << _max_angle
       << " " << _angle_increment
       << " " << _time_increment
       << " " << _scan_time;

    os << " " << _ranges.size() << " ";
    for (size_t i=0; i<_ranges.size(); i++){
      os << _ranges[i] << " ";
    }
    os << " " << _intensities.size() << " ";
    for (size_t i=0; i<_intensities.size(); i++){
      os << _intensities[i] << " ";
    }
  }

  static MessageFactory::MessageRegisterer<LaserMessage> laser_registerer;
  BOSS_REGISTER_CLASS(LaserMessage);
}
