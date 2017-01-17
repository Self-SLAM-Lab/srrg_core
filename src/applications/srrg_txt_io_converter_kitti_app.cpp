#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include "srrg_types/defs.h"
#include "srrg_txt_io/pinhole_image_message.h"

//ds typedefs TODO merge once possible
typedef double real_precision;
typedef Eigen::Transform<real_precision, 3, Eigen::Isometry> TransformMatrix3D;
typedef Eigen::Matrix<real_precision, 3, 3> CameraMatrix;
typedef Eigen::Matrix<real_precision, 3, 4> ProjectionMatrix;
typedef Eigen::Matrix<real_precision, 3, 1> Vector3;

//ds compact KITTI data structure
struct MessageKITTI {

    MessageKITTI(const real_precision& timestamp_,
                 const uint64_t& sequence_number_,
                 const TransformMatrix3D& pose_,
                 const std::string& filename_image_left_,
                 const std::string& filename_image_right_): timestamp(timestamp_),
                                                            sequence_number(sequence_number_),
                                                            pose(pose_),
                                                            filename_image_left(filename_image_left_),
                                                            filename_image_right(filename_image_right_) {}

    real_precision timestamp;
    uint64_t sequence_number;
    TransformMatrix3D pose;
    std::string filename_image_left;
    std::string filename_image_right;
};

//ds entry point
int32_t main(int32_t argc, char** argv) {

  //ds arguments: folder_kitti
  if (argc != 2) {
    std::cerr << "ERROR: invalid call - use ./srrg_txt_io_converter_kitti_app kitti_sequence_number (in the respective kitti sequence folder)" << std::endl;
    std::cerr << "       e.g. ./srrg_txt_io_converter_kitti_app 00" << std::endl;
    return 0;
  }

  //ds raw kitti data folder
  const std::string kitti_sequence_number(argv[1]);

  //ds raw kitti sources
  const std::string filename_images_left("image_0");
  const std::string filename_images_right("image_1");
  const std::string filename_ground_truth(kitti_sequence_number+"_gt.txt");
  const std::string filename_timestamps("times.txt");
  const std::string filename_calibration("calib.txt");

  //ds outfile configuration
  const std::string filename_txt_io(kitti_sequence_number+".txt");
  const std::string folder_txt_io_images(filename_txt_io+".d/");

  //ds dump configuration
  std::cerr << "source  images left: " << filename_images_left << std::endl;
  std::cerr << "       images right: " << filename_images_right << std::endl;
  std::cerr << "       ground truth: " << filename_ground_truth << std::endl;
  std::cerr << "         timestamps: " << filename_timestamps << std::endl;
  std::cerr << "        calibration: " << filename_calibration << std::endl;
  std::cerr << std::endl;
  std::cerr << "txt_io output messages: " << filename_txt_io << std::endl;
  std::cerr << "                images: " << folder_txt_io_images << std::endl;

  //ds attempt to open all files
  std::ifstream file_images_left(filename_images_left.c_str());
  std::ifstream file_images_right(filename_images_right.c_str());
  std::ifstream file_ground_truth(filename_ground_truth.c_str());
  std::ifstream file_timestamps(filename_timestamps.c_str());
  std::ifstream file_calibration(filename_calibration.c_str());
  std::ofstream file_txt_io(filename_txt_io.c_str());

  //ds validate files
  if (!file_images_left.good() || !file_images_left.is_open()) {
    std::cerr << "ERROR: unable to open source for images left: " << filename_images_left << std::endl;
    return 0;
  }
  if (!file_images_right.good() || !file_images_right.is_open()) {
    std::cerr << "ERROR: unable to open source for images right: " << filename_images_right << std::endl;
    return 0;
  }
  if (!file_ground_truth.good() || !file_ground_truth.is_open()) {
    std::cerr << "ERROR: unable to open source for ground truth: " << filename_ground_truth << std::endl;
    return 0;
  }
  if (!file_timestamps.good() || !file_timestamps.is_open()) {
    std::cerr << "ERROR: unable to open source for timestamps: " << filename_timestamps << std::endl;
    return 0;
  }
  if (!file_calibration.good() || !file_calibration.is_open()) {
    std::cerr << "ERROR: unable to open source for calibration: " << filename_calibration << std::endl;
    return 0;
  }
  if (!file_txt_io.good() || !file_txt_io.is_open()) {
    std::cerr << "ERROR: unable to create message file: " << filename_txt_io << std::endl;
    return 0;
  }

  //ds attempt to create the image directory - and check for failure
  if (mkdir(folder_txt_io_images.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0) {
    std::cerr << "ERROR: unable to create image directory: " << folder_txt_io_images << std::endl;
    return 0;
  }

  //ds load projection matrices
  ProjectionMatrix projection_matrix_camera_left(ProjectionMatrix::Zero());
  ProjectionMatrix projection_matrix_camera_right(ProjectionMatrix::Zero());

  //ds two lines
  std::string buffer_projection_matrix_camera_left("");
  std::string buffer_projection_matrix_camera_right("");

  //ds read both lines
  std::getline(file_calibration, buffer_projection_matrix_camera_left);
  std::getline(file_calibration, buffer_projection_matrix_camera_right);

  //ds fill both matrices
  std::istringstream istream_projection_matrix_camera_left(buffer_projection_matrix_camera_left);
  std::string camera_name_left("");
  istream_projection_matrix_camera_left >> camera_name_left;
  for (uint8_t u = 0; u < 3; ++u ) {
    for (uint8_t v = 0; v < 4; ++v ) {
      istream_projection_matrix_camera_left >> projection_matrix_camera_left(u,v);
    }
  }
  std::istringstream istream_projection_matrix_camera_right(buffer_projection_matrix_camera_right);
  std::string camera_name_right("");
  istream_projection_matrix_camera_right >> camera_name_right;
  for (uint8_t u = 0; u < 3; ++u ) {
    for (uint8_t v = 0; v < 4; ++v ) {
      istream_projection_matrix_camera_right >> projection_matrix_camera_right(u,v);
    }
  }

  //ds get the camera matrices
  const CameraMatrix camera_matrix_left(projection_matrix_camera_left.block<3,3>(0,0));
  const CameraMatrix camera_matrix_right(projection_matrix_camera_right.block<3,3>(0,0));

  //ds verify consistency
  if ((camera_matrix_left-camera_matrix_right).norm() != 0) {
    std::cerr << "ERROR: invalid calibration" << std::endl;
  }

  //ds compute offset
  const Vector3 offset(camera_matrix_right.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(projection_matrix_camera_right.block<3,1>(0,3)));
  TransformMatrix3D robot_to_camera_right(TransformMatrix3D::Identity());

  //ds precision thresholding (values very close to zero become zero)
  if (offset.x() > 1e-10 || offset.x() < -1e-10) {
    robot_to_camera_right.translation().x() = offset.x();
  }
  if (offset.y() > 1e-10 || offset.y() < -1e-10) {
    robot_to_camera_right.translation().y() = offset.y();
  }
  if (offset.z() > 1e-10 || offset.z() < -1e-10) {
    robot_to_camera_right.translation().z() = offset.z();
  }

  //ds KITTI data preparation
  std::vector<MessageKITTI> messages_kitti;
  std::string buffer_pose("");
  std::string buffer_timestamp("");

  //ds start reading the files: ground truth/timestamps in parallel
  while (std::getline(file_ground_truth, buffer_pose)   &&
         std::getline(file_timestamps, buffer_timestamp)) {

    //ds parse the pose
    std::istringstream istream_pose(buffer_pose);
    TransformMatrix3D pose(TransformMatrix3D::Identity());
    for (uint8_t u = 0; u < 3; ++u) {
      for (uint8_t v = 0; v < 4; ++v) {
        istream_pose >> pose(u,v);
      }
    }

    //ds generate image numbers
    char buffer_image_number[7];
    const uint64_t& sequence_number = messages_kitti.size();
    std::sprintf(buffer_image_number, "%06lu", sequence_number);
    std::string filename_image_left(filename_images_left+"/"+buffer_image_number+".png");
    std::string filename_image_right(filename_images_right+"/"+buffer_image_number+".png");

    //ds save the message
    messages_kitti.push_back(MessageKITTI(strtod(buffer_timestamp.c_str(), 0), sequence_number, pose, filename_image_left, filename_image_right));
  }

  //ds close all infiles
  file_images_left.close();
  file_images_right.close();
  file_ground_truth.close();
  file_timestamps.close();
  std::cerr << "loaded messages: " << messages_kitti.size() << std::endl;

  //ds generate txt_io messages
  std::cerr << "generating txt_io messages and images: " << std::endl;
  std::cerr << "8";
  srand(time(0));
  for (std::vector<MessageKITTI>::const_iterator message = messages_kitti.begin(); message != messages_kitti.end(); ++message) {

    //ds buffer left and right image
    cv::Mat image_left  = cv::imread(message->filename_image_left, cv::IMREAD_GRAYSCALE);
    cv::Mat image_right = cv::imread(message->filename_image_right, cv::IMREAD_GRAYSCALE);

    if (image_left.rows <= 0 || image_left.cols <= 0) {
      std::cerr << "ERROR: unable to load image left: " << message->filename_image_left << std::endl;
      return 0;
    }
    if (image_right.rows <= 0 || image_right.cols <= 0) {
      std::cerr << "ERROR: unable to load image right: " << message->filename_image_right << std::endl;
      return 0;
    }

    //ds create pinhole messages
    srrg_core::PinholeImageMessage message_left("/camera_left/image_raw", "camera_left", message->sequence_number, message->timestamp);
    srrg_core::PinholeImageMessage message_right("/camera_right/image_raw", "camera_right", message->sequence_number, message->timestamp);

    //ds set images
    message_left.setBinaryFilePrefix(folder_txt_io_images);
    message_right.setBinaryFilePrefix(folder_txt_io_images);
    message_left.setImage(image_left);
    message_right.setImage(image_right);

    //ds set poses
    message_left.setOdometry(message->pose.cast<float>());
    message_right.setOdometry(message->pose.cast<float>());

    //ds set camera matrices
    message_left.setCameraMatrix(camera_matrix_left.cast<float>());
    message_left.setOffset(TransformMatrix3D::Identity().cast<float>());
    message_right.setCameraMatrix(camera_matrix_right.cast<float>());
    message_right.setOffset(robot_to_camera_right.inverse().cast<float>());

    //ds write to stream
    file_txt_io << "PINHOLE_IMAGE_MESSAGE ";
    message_left.toStream(file_txt_io);
    file_txt_io << "\nPINHOLE_IMAGE_MESSAGE ";
    message_right.toStream(file_txt_io);
    file_txt_io << "\n";

    //ds progress info
    if (rand()%2) {
      std::cerr << "=";
    } else {
      std::cerr << "D 8=";
    }
  }
  std::cerr << "D" << std::endl;

  //ds done
  file_txt_io.flush();
  file_txt_io.close();
  return 0;
}
