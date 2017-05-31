#pragma once
#include <string>
#include <iostream>
#include <opencv/cv.h>
#include "base_sensor_message.h"
#include "srrg_types/image_data.h"

namespace srrg_core {
  class BaseImageMessage: public BaseSensorMessage {
  public:
    BaseImageMessage(const std::string& topic="", const std::string& frame_id="", int seq=-1, double timestamp=-1);
    virtual ~BaseImageMessage();
    const cv::Mat& image();
    void setImage(cv::Mat& image_);
    inline void setDepthScale(float depth_scale) {_depth_scale = depth_scale;}
    inline float depthScale() const { return _depth_scale; }

// BOSS interface
    virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);
    virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context);

// TXTIO interface
    virtual const std::string& tag() const;
    inline std::string binaryFilename() const {return  _binaryFilename();}
    virtual void fromStream(std::istream& is);
    virtual void toStream(std::ostream& os) const;
  protected:
    static const std::string _tag;
    virtual void _fetch();
    virtual void _release();
    virtual void _writeBack();
    virtual std::string _binaryFilename() const;
    virtual std::string extension() const;
    float _depth_scale;
    mutable std::string _binary_filename;
    mutable ImageDataBLOBReference _image_reference;
//cv::Mat _image;
  };
}
