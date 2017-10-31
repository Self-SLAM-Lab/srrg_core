/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include <string>

#include "serialization_context.h"
#include "id_context.h"
#include "blob.h"

namespace srrg_boss {

class ObjectWriter;

  class Serializer: virtual public IdContext {
public:
  Serializer(SerializationContext* serContext = 0);
  
  /*
   * Set file path for main data file.
   */
  void setFilePath(const std::string& fpath);
  
  /*
   * Set file path for binary object files (BLOBs).
   * Unless an absolute path is specified the path is relative to the main data file directory.
   * to the directory name, if any.
   */
  void setBinaryPath(const std::string& fpath);
  
  /*
   * Set data file format (default is "JSON").
   */
  bool setFormat(const std::string& format);
  
  /* Write an object.
   * Return false if an error occurred during serialization.
   */
  bool writeObject(Serializable& instance);

  bool writeObject(std::string& output, Serializable& instance);

  virtual ~Serializer();
  
protected:
  ObjectWriter* _objectWriter;
};

}
