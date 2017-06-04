#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "srrg_system_utils/system_utils.h"
#include "srrg_messages/laser_message.h"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_messages/sensor_message_sorter.h"
#include "srrg_boss/deserializer.h"

using namespace std;
using namespace srrg_core;

// Help objects to force linking
PinholeImageMessage i;
LaserMessage l;

const char* banner[] = {
  "srrg_open_boss_file_example_app: example on how to open a txt tio file and read the stuff",
  "",
  "usage: srrg_open_file_example_app <dump_file>",
  0
};

int main(int argc, char ** argv) {
  if (argc < 2 || !strcmp(argv[1], "-h")) {
    printBanner(banner);
    return 0;
  }

  srrg_boss::Deserializer reader;
  reader.setFilePath(argv[1]);

  srrg_boss::Serializable* msg = 0;
  while ((msg = reader.readObject())) {
    BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(msg);
    PinholeImageMessage* pinhole_image_msg=dynamic_cast<PinholeImageMessage*>(msg);
    if (pinhole_image_msg) {
      cv::imshow(pinhole_image_msg->topic(), pinhole_image_msg->image());
      cv::waitKey(1);
    }
    delete msg;
  }
  cerr << "done" << endl;
}
