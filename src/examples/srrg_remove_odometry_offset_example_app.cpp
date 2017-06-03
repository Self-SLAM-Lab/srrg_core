#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "srrg_system_utils/system_utils.h"
#include "srrg_messages/laser_message.h"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/message_writer.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_messages/sensor_message_sorter.h"
#include "srrg_messages/base_sensor_message.h"

using namespace std;
using namespace srrg_core;

const char* banner[] = {
    "srrg_open_file_example_app: example on how to open a txt tio file and read the stuff",
    "",
    "usage: srrg_open_file_example_app <dump_file>",
    0
};

int main(int argc, char ** argv) {
    if (argc < 2 || !strcmp(argv[1], "-h")) {
        printBanner(banner);
        return 0;
    }

    string filename = argv[1];

    MessageReader reader;
    reader.open(filename);

    MessageWriter writer;
    writer.open(filename.substr(0,filename.find("."))+"_corrected.txt");

    Eigen::Isometry3f initial_offset = Eigen::Isometry3f::Identity();
    bool first = true;

    BaseMessage* msg = 0;
    while ((msg = reader.readMessage())) {
        msg->untaint();
        BaseSensorMessage* sensor_msg=dynamic_cast<BaseSensorMessage*>(msg);
        if (sensor_msg){
                if(first){
                    initial_offset = sensor_msg->odometry().inverse();
                    sensor_msg->setOdometry(Eigen::Isometry3f::Identity());
                    first = false;
                    continue;
                }
                sensor_msg->setOdometry(initial_offset*sensor_msg->odometry());
                writer.writeMessage(*sensor_msg);
                cerr << ".";
            }
    }
    cerr << endl << "done" << endl;
}
