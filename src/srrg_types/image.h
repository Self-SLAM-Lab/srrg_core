#pragma once
#include "srrg_boss/blob.h"

namespace srrg_core {
  class ImageData: public srrg_boss::BLOB{
  public:
    virtual bool read(std::istream& is) = 0;
    virtual void write(std::ostream& os) const = 0 ;
    virtual const std::string& extension();
    const cv::Mat& image() const  {return _image;}
    cv::Mat& image() const {return image;}
  protected:
    cv::Mat image;
  };

}
