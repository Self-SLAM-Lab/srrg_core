#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include "srrg_system_utils/system_utils.h"
#include "srrg_messages/laser_message.h"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/message_writer.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_messages/sensor_message_sorter.h"
#include "srrg_messages/spherical_image_message.h"

using namespace std;
using namespace srrg_core;

// Help objects to force linking
PinholeImageMessage im;
SphericalImageMessage sim;
LaserMessage las;

const char* banner[] = {
  "srrg_messages_sorter_app: sorts a txtio file based on timestamps",
  "",
  "usage: srrg_messages_sorter_app -o <output_file> <input",
  0
};



int main(int argc, char ** argv) {
  std::string outfilename="";
  if (argc < 2 || !strcmp(argv[1], "-h")) {
    printBanner(banner);
    return 0;
  }
  int c=1;
  MessageReader reader;
  while (c<argc) {
    if (! strcmp(argv[c],"-o") ){
      c++;
      outfilename=argv[c];
    } else {
      reader.open(argv[c]);
      if (! reader.good()) {
	cerr << "opening file [" << argv[c] <<  "]: ERROR" << endl << "aborting" << endl;
	return -1;
      } else {
	cerr << "opening file [" << argv[c] << "]: OK" << endl;
      }
    }
    c++;
  }

  
  if (! outfilename.size()) {
    cerr << "no outfilename provided, aborting" << endl;
    return -1;
  }

  cerr << "reading messages" << endl;
  std::multimap<double, BaseMessage*> messages;
  
  BaseMessage* msg=0;
  while ((msg=reader.readMessage())) {
    msg->untaint();
    BaseSensorMessage* sensor_msg=dynamic_cast<BaseSensorMessage*>(msg);
    double ts=0;
    if (! sensor_msg) {
      ts=sensor_msg->timestamp();
    }
    messages.insert(std::make_pair(ts,msg));
  }

  MessageWriter writer;
  writer.open(outfilename);
  for (auto it: messages) {
    BaseMessage* msg=it.second;

    // if it is an image message, we load it just to be sure
    // it can be written down
    BaseImageMessage* img_msg=dynamic_cast<BaseImageMessage*>(msg);
    if (img_msg){
      cv::Mat image=img_msg->image();
    }
      
    writer.writeMessage(*msg);
    delete msg;
  }

  writer.close();

  cerr << "done" << endl;
}
