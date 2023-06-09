#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include "srrg_boss/serializer.h"
#include "base_message.h"

namespace srrg_core {
  class MessageWriter {
  public:
    MessageWriter(Format format_ = AUTODETECT);
    ~MessageWriter();

    void open(const std::string& filename);
    void close();
    void writeMessage(BaseMessage& msg);

    inline std::ostream* outputStream() { return _os; }
    inline const std::string& filename() const { return _filename; }
    inline const std::string& binaryFilePrefix() const {return _binary_file_prefix;}
    inline Format format() const {return _format;}
  protected:
    Format _format;
    srrg_boss::Serializer _serializer;
    std::ofstream* _os;
    std::string _filename;
    std::string _binary_file_prefix;

  };
 
}
