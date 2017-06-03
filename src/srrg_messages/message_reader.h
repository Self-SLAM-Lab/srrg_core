#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "base_message.h"
#include "srrg_boss/deserializer.h"

namespace srrg_core {
  class MessageReader {
  public:
    MessageReader(Format fmt=AUTODETECT);
    ~MessageReader();
    void open(const std::string& filename);
    void close();
    BaseMessage* readMessage();
    bool good() const;
    std::istream* inputStream();
    const std::string& filename() const;
    inline Format format() const {return _format;}
  protected:
    srrg_boss::Deserializer _deserializer;
    Format  _format;
    std::ifstream* _is;
    std::string _filename;
  };

}
