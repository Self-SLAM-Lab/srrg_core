#include "base_image_message.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <opencv2/opencv.hpp>

namespace srrg_core {

  using namespace std;

  BaseImageMessage::BaseImageMessage(const std::string& topic_, const std::string& frame_id, int  seq_, double timestamp_):
    BaseSensorMessage(topic_, frame_id, seq_, timestamp_){
    _depth_scale = 1e-3;
  }

  const cv::Mat& BaseImageMessage::image()  {
    fetch();
    ImageData* image_data=_image_reference.get();
    cv::Mat image=image_data->image();
    return image_data->image();
  }

  void BaseImageMessage::setImage(cv::Mat& image_) {
    ImageData* image_data = new ImageData;
    image_data->image() = image_;
    _image_reference.set(image_data);
    _is_fetched=true;
    taint();
  }

  BaseImageMessage::~BaseImageMessage() {
    release();
  }

  std::string BaseImageMessage::_binaryFilename() const {
    if (! _binary_filename.length()) {
      std::string fn  = _topic;
      std::replace( fn.begin(), fn.end(), '/', '.');
      if (fn[0]=='.')
	fn=fn.substr(1);
      char buf[1024];
      extension();
      sprintf(buf,"%s_%08d.%s",fn.c_str(),_seq,extension().c_str());
      _binary_filename = buf;
    }
    return _binary_filename;
  }

  void BaseImageMessage::fromStream(istream& is) {
    BaseSensorMessage::fromStream(is);
    is >> _depth_scale;
    is >> _binary_full_filename;
  }

  void BaseImageMessage::toStream(ostream& os) const {
    BaseSensorMessage::toStream(os);
    os << " " << _depth_scale << " ";
    os << binaryFullFilename();
  }

  void BaseImageMessage::_fetch() {
    const std::string& full_filename = binaryFullFilename();
    const std::string& extension     = full_filename.substr(full_filename.find_last_of(".") + 1);
    ifstream is (full_filename);
    ImageData* image_data=new ImageData();
    image_data->read(is);
    _image_reference.set(image_data);
    untaint();
    cerr << "fetched" << endl;
    /*
    if (extension == "png" || extension =="ppm") {
      _image = cv::imread(full_filename.c_str(), CV_LOAD_IMAGE_COLOR);
      cv::cvtColor(_image, _image, CV_BGR2RGB);
    } else {
      _image = cv::imread(full_filename.c_str(), CV_LOAD_IMAGE_ANYDEPTH);
    }
    */
  }
  
  void BaseImageMessage::_release(){
    _image_reference.set(0);
    //_image.release();
  }

  void BaseImageMessage::_writeBack(){
    std::string full_filename = binaryFullFilename();
    ofstream os (full_filename);
    _image_reference.get()->write(os);
    /*
    //_image.copyTo(image_to_write);
    if (_image.type()==CV_8UC3) {
      cv::Mat image_to_write;
      cv::cvtColor(_image, image_to_write, CV_BGR2RGB);
      cv::imwrite(full_filename.c_str(), image_to_write);
    } else {
      cv::imwrite(full_filename.c_str(), _image);
    }
    */
    untaint();
  }

  void BaseImageMessage::serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    BaseSensorMessage::serialize(data,context);
    if (_depth_scale>0)
      data.setFloat("depth_scale", _depth_scale);
    srrg_boss::ObjectData * blob_data=new srrg_boss::ObjectData();
    data.setField("image", blob_data);
    _image_reference.setNameAttribute(_binaryFilename());
    _image_reference.serialize(*blob_data,context);
    untaint();
  }
  
  void BaseImageMessage::deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) {
    BaseSensorMessage::deserialize(data,context);
    _depth_scale=0;
    if (data.getField("depth_scale")){
      _depth_scale=data.getFloat("depth_scale");
    }
    srrg_boss::ObjectData * blob_data = static_cast<srrg_boss::ObjectData *>(data.getField("image"));
    _image_reference.deserialize(*blob_data,context);
    _binary_filename=_image_reference.nameAttribute();
    untaint();
    _is_fetched=true; // let boss handle the fetching
  }

  const std::string BaseImageMessage::_tag="BASE_IMAGE_MESSAGE";

  const std::string& BaseImageMessage::tag() const { return _tag; }

  std::string BaseImageMessage::extension() const {
    return _image_reference.get()->const_extension();
    // if (_image.type()==CV_16UC1||_image.type()==CV_8UC1) {
    //    return "pgm";
    // } else if (_image.type()==CV_8UC3) {
    //   return  "png";
    // } else {
    //   cerr << " image type: " << _image.type() << "rows x cols: " << _image.rows << "x" << _image.cols << endl;
    //   throw std::runtime_error("Unknown extension for image of this type");
    // }
  }

  BOSS_REGISTER_CLASS(BaseImageMessage);

}
