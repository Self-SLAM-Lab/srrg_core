#pragma once
#include "srrg_boss/blob.h"
#include "defs.h"

namespace srrg_core {
  class ImageData: public srrg_boss::BLOB{
  public:
    virtual bool read(std::istream& is);
    virtual void write(std::ostream& os) const ;
    virtual const std::string& extension();
    const cv::Mat& image() const  {return _image;}
    cv::Mat& image() {return _image;}
    void setImage(cv::Mat& image_){image_.copyTo(_image);}
    const std::string& const_extension() const;
  protected:
    cv::Mat _image;
  };

  typedef srrg_boss::BLOBReference<ImageData> ImageDataBLOBReference;

}
