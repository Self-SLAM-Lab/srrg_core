#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include "srrg_system_utils/system_utils.h"
#include "srrg_messages/message_reader.h"
#include "srrg_messages/message_writer.h"
#include "srrg_messages/base_sensor_message.h"

using namespace std;
using namespace srrg_core;

//typedef std::map<int,Eigen::Isometry3f> TrajectoryMap;
typedef std::map<double,Eigen::Isometry3f> TrajectoryMap;

const char* banner[] = {
    "srrg_trajectory_saver_example: example on how to save a trajectory in a txt_io file",
    "",
    "usage: srrg_trajectory_saver_example <trajectory_file> <dump_file>",
    0
};

int main(int argc, char ** argv) {
    if (argc < 2 || !strcmp(argv[1], "-h")) {
        printBanner(banner);
        return 0;
    }

    string trajectory_filename = argv[1];
    string txt_io_filename = argv[2];
    string topic_name = "/xtion/depth/image_raw";

    MessageReader reader;
    reader.open(txt_io_filename.c_str());

    ifstream trajectory_file;
    trajectory_file.open(trajectory_filename.c_str());

    if(reader.good())
        cerr << "Opened dump file: " << txt_io_filename << endl;
    else
        cerr << "Error opening file: " << txt_io_filename << endl;

    if(trajectory_file.is_open())
        cerr << "Opened trajectory file: " << trajectory_filename << endl;
    else
        cerr << "Error opening file: " << trajectory_filename << endl;


    TrajectoryMap trajectory_map;
    string line;
    while(getline(trajectory_file,line)) {
        istringstream iss(line);
        //int seq;
        double timestamp;
        float x,y,theta;
        //iss >> seq
        iss >> timestamp
            >> x >> y >> theta;

        Eigen::Isometry3f odometry = Eigen::Isometry3f::Identity();
        odometry.translation() = Eigen::Vector3f (x,y,0);
        odometry.rotate(Eigen::AngleAxisf (theta,Eigen::Vector3f::UnitZ()));

        //trajectory_map.insert(std::pair<int,Eigen::Isometry3f> (seq,odometry));
        trajectory_map.insert(std::pair<double,Eigen::Isometry3f> (timestamp,odometry));
    }
    cerr << "Saving trajectory for " << trajectory_map.size() << " messages on topic: " << topic_name << endl;

    MessageWriter writer;
    writer.open(txt_io_filename.substr(0,txt_io_filename.find("."))+"_with_trajectory.txt");
    BaseMessage* msg = 0;

    double tolerance = 0.01;

    for(TrajectoryMap::iterator it = trajectory_map.begin(); it != trajectory_map.end();it++){
        //int seq = it->first;
        double timestamp = it->first;
        Eigen::Isometry3f current_transform = it->second;
        bool found = false;
        while(!found && (msg = reader.readMessage())){
            BaseSensorMessage* sensor_msg = dynamic_cast<BaseSensorMessage*>(msg);
            if (sensor_msg){
                //if(sensor_msg->topic() == topic_name && sensor_msg->seq() == seq)
                if(sensor_msg->topic() == topic_name && fabs(sensor_msg->timestamp()-timestamp)<tolerance){
                    sensor_msg->setOdometry(current_transform);
                    cerr << ".";
                    found = true;
                    writer.writeMessage(*msg);
                }
            }
        }
    }
    cerr << endl << "done" << endl;
}
