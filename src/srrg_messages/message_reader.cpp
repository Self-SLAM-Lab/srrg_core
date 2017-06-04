#include <string>
#include <iostream>
#include <fstream>
#include "message_reader.h"
#include <sstream>
#include "message_factory.h"

namespace srrg_core {
  using namespace std;

  MessageReader::MessageReader(Format format_) {
    _format=format_;
    _is = 0;
    _filename = "";
  }
  
  #define MAX_BUF_SIZE 65535

  BaseMessage* MessageReader::readMessage(){
    switch (_format) {
    case TXTIO:
      {
	if (! _is)
	  return 0;
	char buf[MAX_BUF_SIZE];
	_is->getline(buf, MAX_BUF_SIZE);
	istringstream is(buf);
	std::string tag;
	is >> tag;
	BaseMessage* m = MessageFactory::instance()->create(tag);
	if (!m)
	  return 0;
	m->fromStream(is);
	return m;
      }
    case BOSS:
      {
	srrg_boss::Serializable* o=_deserializer.readObject();
	if (!o)
	  return 0;
	BaseMessage* msg=dynamic_cast<BaseMessage*>(o);
	return msg;
      }
    }
  }

  bool MessageReader::good() const {
    switch (_format) {
    case TXTIO:
      if (! _is)
	return false;
      if (! _is->good())
	return false;
      return true;
    case BOSS:
      return true; // HACK
    }
  }

  void MessageReader::open(const std::string& filename_){
    _filename=filename_;

    if (_format==AUTODETECT) {
      std::string extension=_filename.substr(_filename.find_last_of(".") + 1);
      if (extension=="txt" || extension=="txtio"){
	_format=TXTIO;
      } else if(extension=="boss") {
	_format=BOSS;
      }
    }
    if (_format==AUTODETECT)
      throw std::runtime_error("invalid serialization format, check the extension");

    switch (_format) {
    case TXTIO:
      if (_is){
	delete _is;
      }
      _is = new std::ifstream(_filename.c_str());
      if (! _is->good()){
	delete _is;
	_is = 0;
      }
    case BOSS:
      _deserializer.setFilePath(_filename);
    } 
    
  }
    
  void MessageReader::close(){
    switch(_format) {
    case TXTIO:
      if (_is){
	delete _is;
      }
      _is = 0;
      _filename = "";
      break;
    case BOSS:
      ;
    }
  }

  MessageReader
  ::~MessageReader() {
    close();
  }


  inline std::istream* MessageReader::inputStream() {
    if (_format==BOSS)
      throw std::runtime_error("can't get the stream in BOSS mode");
    return _is;
  }
  
  inline const std::string& MessageReader::filename() const { return _filename; }

  
}
