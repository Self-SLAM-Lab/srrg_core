#pragma once
#include "sensor_message_sorter.h"
#include "message_writer.h"
#include "srrg_boss/serializer.h"

namespace srrg_core {

  class MessageDumperTrigger: public SensorMessageSorter::Trigger{
  public:
    MessageDumperTrigger(SensorMessageSorter* sorter,
			 int priority,
			 MessageWriter* writer, 
			 const std::string& file_prefix="");

    MessageDumperTrigger(SensorMessageSorter* sorter,
			 int priority,
			 srrg_boss::Serializer* serializer);

    virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg);

  protected:
    MessageWriter* _writer;
    srrg_boss::Serializer* _serializer;
    std::string _file_prefix;
  };
}
