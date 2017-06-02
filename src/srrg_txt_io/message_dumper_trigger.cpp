#include "message_dumper_trigger.h"
#include "srrg_types/defs.h"

namespace srrg_core {
  
  using namespace std;

  MessageDumperTrigger::MessageDumperTrigger(SensorMessageSorter* sorter,
					     int priority,
					     MessageWriter* writer, 
					     const std::string& file_prefix) :
    SensorMessageSorter::Trigger(sorter, priority){
    _writer = writer;
    _serializer = 0;
    _file_prefix = file_prefix;
  }

    MessageDumperTrigger::MessageDumperTrigger(SensorMessageSorter* sorter,
					       int priority,
					       srrg_boss::Serializer* serializer) :
    SensorMessageSorter::Trigger(sorter, priority){
    _writer = 0;
    _serializer=serializer;
    _file_prefix = "";
  }

  void MessageDumperTrigger::action(std::tr1::shared_ptr<BaseSensorMessage> msg){
    if (_writer) {
      _writer->writeMessage(*msg);
      msg->writeBack();
    }
    if (_serializer){
      msg->untaint();
      _serializer->writeObject(*msg);
    }
  }

  
}
