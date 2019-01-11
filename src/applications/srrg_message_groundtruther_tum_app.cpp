#include <srrg_messages/message_reader.h>
#include <srrg_messages/message_writer.h>
#include <srrg_messages/pinhole_image_message.h>
#include <srrg_messages/sensor_message_sorter.h>
#include <srrg_messages/message_timestamp_synchronizer.h>

#include <srrg_system_utils/system_utils.h>

#include <sstream>

using namespace srrg_core;

typedef std::map<double, Vector6f, std::less<double>, Eigen::aligned_allocator<Vector6f>> TimePoseMap;
typedef std::pair<double, Vector6f> TimePosePair;


void loadGT(TimePoseMap& time_pose, const std::string gt_filename);
Vector6f getClosestPose(const TimePoseMap& time_pose, const double& current_time);


const char* banner [] = {
  "srrg_message_groundtruther_tum_app for txtio files",
  "",
  "this app adds the data that you pass through the <odometry-gt.txt> file to the txtio dataset selected",
  "<odometry-gt.txt> can contain either ground thruth or odometry but must be in TUM format",
  "that data will populate the <odom> field of the ImageMessage",
  "usage: srrg_message_groundtruther_tum_app <depth-topic> <rgb-topic> <odometry-gt.txt> <input.txtio> <output.txtio>",
  0
};


int main(int argc, char** argv) {

  if (argc < 6 || !strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
    srrg_core::printBanner(banner);
    throw std::runtime_error("exiting");
  }

  // const std::string depth_topic = "/camera/depth/image";
  // const std::string rgb_topic = "/camera/rgb/image_color";
  std::string depth_topic(argv[1]);
  std::string rgb_topic(argv[2]);
  std::string groundtruth(argv[3]);
  std::string filename(argv[4]);
  std::string output_filename(argv[5]);
  const float time_interval = 0.03;
  
  srrg_core::MessageReader reader(TXTIO);
  srrg_core::MessageWriter writer(TXTIO);
  srrg_core::MessageTimestampSynchronizer synchronizer;

  std::vector<std::string> depth_plus_rgb_topic;
  depth_plus_rgb_topic.push_back(depth_topic);
  depth_plus_rgb_topic.push_back(rgb_topic);
  synchronizer.setTopics(depth_plus_rgb_topic);
  synchronizer.setTimeInterval(time_interval);

  TimePoseMap time_pose_map;
  loadGT(time_pose_map, groundtruth);
  reader.open(filename);

  writer.open(output_filename);
  
  BaseMessage* msg_base = 0;    
  while ((msg_base=reader.readMessage())) {
    msg_base->untaint();
    if(!msg_base)
      throw std::runtime_error("[MsgBase unrecognized]");
    BaseImageMessage* msg_img = dynamic_cast<BaseImageMessage*>(msg_base);
    if(!msg_img)
      throw std::runtime_error("[BaseImgMsg unrecognized]");

    synchronizer.putMessage(msg_img);
    if (!synchronizer.messagesReady())
      continue;

    PinholeImageMessage* msg_img_depth = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[0].get());;
    PinholeImageMessage* msg_img_rgb = dynamic_cast<PinholeImageMessage*>(synchronizer.messages()[1].get());
    double timestamp = msg_img_rgb->timestamp();
    Vector6f gt_pose =  getClosestPose(time_pose_map, timestamp);

    msg_img_depth->setOdometry(srrg_core::v2t(gt_pose));
    msg_img_rgb->setOdometry(srrg_core::v2t(gt_pose));
    writer.writeMessage(*msg_img_depth);
    writer.writeMessage(*msg_img_rgb);
  }

  reader.close();
  writer.close();
  
  return 0;
}


void loadGT(TimePoseMap& time_pose, const std::string gt_filename) {
  std::ifstream infile(gt_filename);

  std::string line;
  size_t k = 0;
  while (std::getline(infile, line))
    {
      // jump first three lines
      if(k++ < 3) 
        continue;
      std::istringstream iss(line);
      double timestamp;
      float x, y, z, qx, qy, qz, qw;
      if (!(iss
            >> timestamp
            >> x
            >> y
            >> z
            >> qx
            >> qy
            >> qz
            >> qw))
        {std::cerr << "[LoadGT]: WRONG LINE" << std::endl;
          break; }
      Vector6f pose;
      pose << x, y, z, qx, qy, qz;
      if(qw < 0)
        pose.tail(3)*=-1;
      //std::cerr << timestamp << " " << pose.transpose() << std::endl;
      time_pose.insert(TimePosePair(timestamp, pose));
    }
  
}

Vector6f getClosestPose(const TimePoseMap& time_pose, const double& current_time) {
  TimePoseMap::const_iterator it = time_pose.lower_bound(current_time);
  if(it != time_pose.begin()) {
    --it;
  } else {
    std::cerr << "[getClosestPose]: Missing Some GT. Adding the first/closest" << std::endl;
  }
  return it->second;

}
