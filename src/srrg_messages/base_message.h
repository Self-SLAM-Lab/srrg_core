#pragma once
#include <string>
#include <iostream>
#include "srrg_boss/serializable.h"

namespace srrg_core {

  enum Format {AUTODETECT=0x0, BOSS=0x1, TXTIO=0x2};

  class BaseMessage: public srrg_boss::Serializable {
  public:
    BaseMessage();
    virtual ~BaseMessage();
    virtual const std::string& tag() const = 0;
    virtual void fromStream(std::istream& is);
    virtual void  toStream(std::ostream& os) const;
    void fetch();
    void release(); 
    void writeBack();
    inline void taint() {_is_tainted = true; }
    inline void untaint() {_is_tainted = false; }
    inline const std::string& binaryFilePrefix() const {return _binary_file_prefix;}
    inline void setBinaryFilePrefix(const std::string& bup) const {_binary_file_prefix=bup;}
    std::string binaryFullFilename() const;
  protected:
    virtual std::string _binaryFilename(bool recompute=false) const;
    virtual void _fetch();
    virtual void _release();
    virtual void _writeBack();
    
    mutable std::string _binary_file_prefix;
    mutable std::string _binary_full_filename;
    bool _is_fetched;
    bool _is_tainted;
  };
}
