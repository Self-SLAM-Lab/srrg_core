#include "image_data.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdexcept>

const std::string png_extension = "png";
const std::string pgm_extension  = "pgm";

  
namespace srrg_core {
  bool ImageData::read(std::istream& is) {
    std::vector<unsigned char> buf;
    const int block_size=4096;
    int bytes_read=0;
    while(is){
      size_t old_size=buf.size();
      buf.resize(buf.size()+block_size);
      is.read(reinterpret_cast<char*>(&buf[old_size]), block_size);
      bytes_read+=is.gcount();
    }
    buf.resize(bytes_read);
    _image = cv::imdecode(buf,CV_LOAD_IMAGE_ANYDEPTH);
  }

  void ImageData::write(std::ostream& os) const {
    std::vector<unsigned char> buf;
    const cv::Mat const_image=_image;
    cv::imencode(const_extension(), const_image, buf);
    os.write(reinterpret_cast<char*>(&buf[0]), buf.size());
  }

  const std::string& ImageData::extension() {
    return const_extension();
  }

  const std::string& ImageData::const_extension() const  {
    switch(_image.type()){
    case CV_8UC3:
    case CV_8UC1:
      return png_extension;
    case CV_16UC1:
      return pgm_extension;
    default:
      throw std::runtime_error("unknown image type");
    }
  }
}
