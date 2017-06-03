#include <string>
#include <iostream>
#include <fstream>
#include "message_writer.h"
#include <sys/types.h>
#include <sys/stat.h>

namespace srrg_core {
  using namespace std;

  MessageWriter::MessageWriter(Format format_) {
    _format=format_;
    _os = 0;
    _filename = "";
    _binary_file_prefix = "";
  }

  MessageWriter::~MessageWriter() {
  }

  void MessageWriter::open(const std::string& filename_) {
    close();
    _filename = filename_;

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
    
    _binary_file_prefix = _filename+".d";
    if (_format==TXTIO) {
      mkdir(_binary_file_prefix.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
      _binary_file_prefix = _filename+".d/";
      _os = new ofstream(_filename.c_str());
    } else {
      _serializer.setFilePath(_filename);
      _serializer.setBinaryPath(_filename+".d/<nameAttribute>");
    }
  }


  void MessageWriter::close() {
    if (_format==TXTIO) {
      if (_os)
	delete _os;
      _os=0;
      _filename = "";
      _binary_file_prefix = "";
    }
  }
  
  void MessageWriter::writeMessage(BaseMessage& msg) {
    if (_format==TXTIO) {
      if (!_os)
	return;
      msg.setBinaryFilePrefix(_binary_file_prefix);
      *_os << msg.tag() << " ";
      msg.toStream(*_os);
      * _os << endl;
    } else {
      _serializer.writeObject(msg);
    }
  }

}
