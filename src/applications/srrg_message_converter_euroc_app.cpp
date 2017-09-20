#include <sys/stat.h>
#include "srrg_messages/message_writer.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_messages/imu_message.h"

//ds opencv
#include <opencv2/opencv.hpp>

using namespace srrg_core;

//ds desired precision (internal opencv operations are done in big floats/doubles)
typedef double real;
typedef Eigen::Matrix<real, 3, 3> Matrix3;
typedef Eigen::Matrix<real, 3, 1> Vector3;
typedef Eigen::Quaternion<real> Quaternion;
typedef Eigen::Matrix<real, 3, 4> ProjectionMatrix;
typedef Eigen::Transform<real, 3, Eigen::Isometry> TransformMatrix3D;

//ds refined ground truth fused with IMU
struct MeasurementGTIMU {

  MeasurementGTIMU(const uint64_t& timestamp_milliseconds_,
                   const TransformMatrix3D& imu_to_world_): timestamp_milliseconds(timestamp_milliseconds_),
                                                            imu_to_world(imu_to_world_) {}

  //ds available values
  //p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
  //q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],
  //v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],
  //b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],
  //b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]

  //ds TODO extend attributes
  uint64_t timestamp_milliseconds;
  TransformMatrix3D imu_to_world;
};

//ds C++11 not required
struct CustomComparator {
  bool operator() (const std::pair<uint64_t, std::string>& a_, const std::pair<uint64_t, std::string>& b_) { return (a_.first < b_.first);}
} CustomComparator;

//ds wrappers - without need to add the YAML library
void loadParametersCamera(const std::string& filename_camera_,
                          TransformMatrix3D& camera_to_imu_,
                          cv::Size& image_size_,
                          Matrix3& camera_matrix_,
                          cv::Mat& distortion_coefficients_) {

  //ds load configuration files
  std::ifstream file_camera_left(filename_camera_.c_str());

  //ds get the whole file as string
  std::string data_camera_left;
  file_camera_left.seekg(0, std::ios::end);
  data_camera_left.reserve(file_camera_left.tellg());
  file_camera_left.seekg(0, std::ios::beg);
  data_camera_left.assign((std::istreambuf_iterator<char>(file_camera_left)),
                           std::istreambuf_iterator<char>());

  //ds set transform camera to imu
  const std::string::size_type index_begin_transform  = data_camera_left.find("data: [") + strlen("data: [");
  const std::string::size_type index_end_transform    = data_camera_left.find("]", index_begin_transform);
  const std::string data_transform = data_camera_left.substr(index_begin_transform, index_end_transform-index_begin_transform);
  std::string::size_type index_begin_item = 0;
  std::string::size_type index_end_item   = 0;
  TransformMatrix3D camera_to_imu(TransformMatrix3D::Identity());
  for (uint32_t row = 0; row < 4; ++row) {
    for (uint32_t col = 0; col < 4; ++col) {
      index_end_item = data_transform.find(",", index_begin_item);
      camera_to_imu(row, col) = std::strtod(data_transform.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
      index_begin_item = index_end_item+2;
    }
  }

  //ds compute transform camera to robot from camera to imu (coordinate system flip to world frame)
  camera_to_imu_ = camera_to_imu;

  //ds set image info
  const std::string::size_type index_begin_resolution = data_camera_left.find("resolution: [") + strlen("resolution: [");
  const std::string::size_type index_end_resolution   = data_camera_left.find("]", index_begin_resolution);
  const std::string data_resolution = data_camera_left.substr(index_begin_resolution, index_end_resolution-index_begin_resolution);
  const std::string::size_type index_end_width = data_resolution.find(",");
  image_size_.width  = std::strtod(data_resolution.substr(0, index_end_width).c_str(), 0);
  image_size_.height = std::strtod(data_resolution.substr(index_end_width+2).c_str(), 0);

  //ds set camera matrix
  const std::string::size_type index_begin_intrinsics = data_camera_left.find("intrinsics: [") + strlen("intrinsics: [");
  const std::string::size_type index_end_intrinsics   = data_camera_left.find("]", index_begin_intrinsics);
  const std::string data_intrinsics = data_camera_left.substr(index_begin_intrinsics, index_end_intrinsics-index_begin_intrinsics);

  //ds set Fx, Fy, Cx, Cy... lets unroll this loop for the compiler
  index_begin_item = 0;
  index_end_item   = data_intrinsics.find(",", index_begin_item);
  camera_matrix_(0,0) = std::strtod(data_intrinsics.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
  index_begin_item = index_end_item+2;
  index_end_item   = data_intrinsics.find(",", index_begin_item);
  camera_matrix_(1,1) = std::strtod(data_intrinsics.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
  index_begin_item = index_end_item+2;
  index_end_item   = data_intrinsics.find(",", index_begin_item);
  camera_matrix_(0,2) = std::strtod(data_intrinsics.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
  index_begin_item = index_end_item+2;
  index_end_item   = data_intrinsics.find(",", index_begin_item);
  camera_matrix_(1,2) = std::strtod(data_intrinsics.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);

  //ds set distortion coefficients
  const std::string::size_type index_begin_distortion = data_camera_left.find("distortion_coefficients: [") + strlen("distortion_coefficients: [");
  const std::string::size_type index_end_distortion   = data_camera_left.find("]", index_end_distortion);
  const std::string data_distortion = data_camera_left.substr(index_begin_distortion, index_end_distortion-index_begin_distortion);
  index_begin_item = 0;
  for (uint32_t row = 0; row < 4; ++row) {
    index_end_item = data_distortion.find(",", index_begin_item);
    distortion_coefficients_.row(row) = std::strtod(data_distortion.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
    index_begin_item = index_end_item+2;
  }
}

std::map<uint64_t, MeasurementGTIMU> loadGroundTruthForIMU(const std::string& filename_ground_truth_) {

  //ds load data file (comma separated values)
  std::ifstream file_ground_truth(filename_ground_truth_.c_str());

  //ds output map
  std::map<uint64_t, MeasurementGTIMU> ground_truth;

  //ds read line by line
  std::string buffer_line;
  while (std::getline(file_ground_truth, buffer_line)) {

    //ds skip comment lines
    if (buffer_line[0] == '#') {
      continue;
    }

    //ds parse control
    std::string::size_type index_begin_item = 0;
    std::string::size_type index_end_item   = 0;

    //ds parse timestamp - cutting off the 6 last digits (from nanoseconds to milliseconds)
    index_end_item = buffer_line.find(",", index_begin_item);
    const uint64_t timestamp_milliseconds = std::atol(buffer_line.substr(index_begin_item, index_end_item-6).c_str());
    index_begin_item = index_end_item+1;

    //ds position buffer
    TransformMatrix3D imu_to_world(TransformMatrix3D::Identity());
    for (uint32_t row = 0; row < 3; ++row) {
      index_end_item = buffer_line.find(",", index_begin_item);
      imu_to_world.translation()(row) = std::strtod(buffer_line.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
      index_begin_item = index_end_item+1;
    }

    //ds buffer quaternion
    Quaternion imu_to_world_rotation;
    index_end_item = buffer_line.find(",", index_begin_item);
    imu_to_world_rotation.w() = std::strtod(buffer_line.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
    index_begin_item = index_end_item+1;
    index_end_item = buffer_line.find(",", index_begin_item);
    imu_to_world_rotation.x() = std::strtod(buffer_line.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
    index_begin_item = index_end_item+1;
    index_end_item = buffer_line.find(",", index_begin_item);
    imu_to_world_rotation.y() = std::strtod(buffer_line.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
    index_begin_item = index_end_item+1;
    index_end_item = buffer_line.find(",", index_begin_item);
    imu_to_world_rotation.z() = std::strtod(buffer_line.substr(index_begin_item, index_end_item-index_begin_item).c_str(), 0);
    index_begin_item = index_end_item+1;
    imu_to_world.linear() = imu_to_world_rotation.toRotationMatrix();

    //ds add to buffer
    ground_truth.insert(std::make_pair(timestamp_milliseconds, MeasurementGTIMU(timestamp_milliseconds, imu_to_world)));
  }

  return ground_truth;
}

std::vector<std::pair<uint64_t, std::string> > loadImages(const std::string& folder_images_) {

  //ds stamped images
  std::vector<std::pair<uint64_t, std::string> > images;

  //ds load data file (comma separated values)
  std::ifstream file_images((folder_images_+"/data.csv").c_str());

  //ds image path
  const std::string image_path(folder_images_+"/data/");

  //ds read line by line
  std::string buffer_line;
  while (std::getline(file_images, buffer_line)) {

    //ds skip comment lines
    if (buffer_line[0] == '#') {
      continue;
    }

    //ds parse control
    std::string::size_type index_begin_item = 0;
    std::string::size_type index_end_item   = 0;

    //ds parse timestamp
    index_end_item = buffer_line.find(",", index_begin_item);
    const uint64_t timestamp_nanoseconds = std::atol(buffer_line.substr(index_begin_item, index_end_item).c_str());
    index_begin_item = index_end_item+1;

    //ds parse image name and store it
    images.push_back(std::make_pair(timestamp_nanoseconds, image_path+buffer_line.substr(index_begin_item, buffer_line.length()-index_begin_item-1)));
  }

  return images;
}

//ds entry point
int32_t main(int32_t argc, char** argv) {

  //ds arguments at least 2 - optional topic names
  if (argc < 3) {
    std::cerr << "ERROR: invalid call - use ./srrg_messages_converter_euroc_app -o <messages.txt> [-parse-gt -hard-calibration]" << std::endl;
    return 0;
  }

  //ds obtain configuration
  std::string outfile_name               = "";
  std::string topic_name_camera_left  = "/cam0/image_raw";
  std::string topic_name_camera_right = "/cam1/image_raw";
  std::string topic_name_imu          = "/imu0";
  bool option_convert_ground_truth    = true;
  bool option_use_orbslam_calibration = false;

  //ds parse parameters
  int32_t count_added_arguments = 1;
  while(count_added_arguments < argc){
    if (!strcmp(argv[count_added_arguments],"-o")) {
      count_added_arguments++;
      outfile_name = argv[count_added_arguments];
    } else if (!strcmp(argv[count_added_arguments],"-parse-gt")) {
      option_convert_ground_truth = true;
      option_use_orbslam_calibration = false;
    } else if (!strcmp(argv[count_added_arguments],"-hard-calibration")) {
      option_use_orbslam_calibration = true;
      option_convert_ground_truth = false;
    }
    count_added_arguments++;
  }

  //ds if elementary input is not set
  if (outfile_name == "") {
    std::cerr << "ERROR: parameter -o <messages.txt> not set" << std::endl;
    return 0;
  }

  //ds log configuration
  std::cerr << "topic_name_camera_left: " << topic_name_camera_left << std::endl;
  std::cerr << "topic_name_camera_right: " << topic_name_camera_right << std::endl;
  std::cerr << "topic_name_imu: " << topic_name_imu << std::endl;
  std::cerr << "outfile_name: " << outfile_name << std::endl;
  std::cerr << "option_convert_ground_truth: " << option_convert_ground_truth << std::endl;
  std::cerr << "option_use_orbslam_calibration: " << option_use_orbslam_calibration << std::endl;

  //ds load images
  std::vector<std::pair<uint64_t, std::string> > images_left  = loadImages("cam0");
  std::vector<std::pair<uint64_t, std::string> > images_right = loadImages("cam1");
  std::cerr << "loaded images  left: " << images_left.size() << std::endl;
  std::cerr << "loaded images right: " << images_right.size() << std::endl;

  //ds parameter pool
  cv::Size image_size(752, 480);
  cv::Mat camera_calibration_matrix_left(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat camera_calibration_matrix_right(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat distortion_coefficients_left(cv::Mat::zeros(4, 1, CV_64F));
  cv::Mat distortion_coefficients_right(cv::Mat::zeros(4, 1, CV_64F));
  cv::Mat projection_matrix_left(cv::Mat::eye(3, 4, CV_64F));
  cv::Mat projection_matrix_right(cv::Mat::eye(3, 4, CV_64F));
  cv::Mat rectification_matrix_left(cv::Mat::eye(3, 3, CV_64F));
  cv::Mat rectification_matrix_right(cv::Mat::eye(3, 3, CV_64F));
  TransformMatrix3D camera_left_to_right(TransformMatrix3D::Identity());

  //ds ground truth measurements (optional)
  std::map<uint64_t, MeasurementGTIMU> ground_truth;
  TransformMatrix3D camera_left_to_imu(TransformMatrix3D::Identity());
  TransformMatrix3D camera_right_to_imu(TransformMatrix3D::Identity());
  if (option_convert_ground_truth) {

    //ds parameter pool: camera matrices
    Matrix3 camera_matrix_left_eigen(Matrix3::Identity());
    Matrix3 camera_matrix_right_eigen(Matrix3::Identity());

  //  //ds coordinate system adjustment (so our robot has its z axis aligned with the world z axis)
  //  TransformMatrix3D imu_to_robot(TransformMatrix3D::Identity());
  //  imu_to_robot.linear() << 0, 0, 1,
  //                           1, 0, 0,
  //                           0, 1, 0;
  //  const TransformMatrix3D robot_to_imu(imu_to_robot.inverse());

    //ds load configuration files - setting the parameter pool
    loadParametersCamera("cam0/sensor.yaml", camera_left_to_imu, image_size, camera_matrix_left_eigen, distortion_coefficients_left);
    loadParametersCamera("cam1/sensor.yaml", camera_right_to_imu, image_size, camera_matrix_right_eigen, distortion_coefficients_right);

    //ds load ground truth poses
    ground_truth = loadGroundTruthForIMU("state_groundtruth_estimate0/data.csv");
    std::cerr << "loaded ground truth poses: " << ground_truth.size() << std::endl;

    //ds compute relative transform
    camera_left_to_right = camera_right_to_imu.inverse()*camera_left_to_imu;

    //ds rectification
    cv::Mat rotation_camera_left_to_right(3, 3, CV_64F, cv::Scalar(0));
    cv::Mat translation_camera_left_to_right(3, 1, CV_64F, cv::Scalar(0));
    cv::Mat depth_mapping(4, 4, CV_64F, cv::Scalar(0));

    //ds buffer matrices to opencv
    for (int32_t u = 0; u < 3; ++u) {
      for (int32_t v = 0; v < 3; ++v) {
        camera_calibration_matrix_left.at<double>(u, v)  = camera_matrix_left_eigen(u,v);
        camera_calibration_matrix_right.at<double>(u, v) = camera_matrix_right_eigen(u,v);
        rotation_camera_left_to_right.at<double>(u, v)   = camera_left_to_right.linear()(u,v);
      }
      translation_camera_left_to_right.row(u) = camera_left_to_right.translation()(u);
    }

    //ds compute rectification parameters
    cv::stereoRectify(camera_calibration_matrix_left,
                      distortion_coefficients_left,
                      camera_calibration_matrix_right,
                      distortion_coefficients_right,
                      image_size,
                      rotation_camera_left_to_right,
                      translation_camera_left_to_right,
                      rectification_matrix_left,
                      rectification_matrix_right,
                      projection_matrix_left,
                      projection_matrix_right,
                      depth_mapping,
                      CV_CALIB_ZERO_DISPARITY,
                      0);

    //ds check if failed
    if (cv::norm(projection_matrix_left) == 0 || cv::norm(projection_matrix_right) == 0) {
      std::cerr << "ERROR: rectification parameter retrieval failed" << std::endl;
      return 0;
    }
  } else if (option_use_orbslam_calibration) {

    camera_calibration_matrix_left.at<double>(0,0)  = 458.654;
    camera_calibration_matrix_left.at<double>(0,2)  = 367.215;
    camera_calibration_matrix_left.at<double>(1,1)  = 457.296;
    camera_calibration_matrix_left.at<double>(1,2)  = 248.375;
    camera_calibration_matrix_right.at<double>(0,0) = 457.587;
    camera_calibration_matrix_right.at<double>(0,2) = 379.999;
    camera_calibration_matrix_right.at<double>(1,1) = 456.134;
    camera_calibration_matrix_right.at<double>(1,2) = 255.238;

    distortion_coefficients_left.at<double>(0)  = -0.28340811;
    distortion_coefficients_left.at<double>(1)  = 0.07395907;
    distortion_coefficients_left.at<double>(2)  = 0.00019359;
    distortion_coefficients_left.at<double>(3)  = 1.76187114e-05;
    distortion_coefficients_right.at<double>(0) = -0.28368365;
    distortion_coefficients_right.at<double>(1) = 0.07451284;
    distortion_coefficients_right.at<double>(2) = -0.00010473;
    distortion_coefficients_right.at<double>(3) = -3.55590700e-05;

    rectification_matrix_left.at<double>(0,0) = 0.999966347530033;
    rectification_matrix_left.at<double>(0,1) = -0.001422739138722922;
    rectification_matrix_left.at<double>(0,2) = 0.008079580483432283;
    rectification_matrix_left.at<double>(1,0) = 0.001365741834644127;
    rectification_matrix_left.at<double>(1,1) = 0.9999741760894847;
    rectification_matrix_left.at<double>(1,2) = 0.007055629199258132;
    rectification_matrix_left.at<double>(2,0) = -0.008089410156878961;
    rectification_matrix_left.at<double>(2,1) = -0.007044357138835809;
    rectification_matrix_left.at<double>(2,2) = 0.9999424675829176;

    rectification_matrix_right.at<double>(0,0) = 0.9999633526194376;
    rectification_matrix_right.at<double>(0,1) = -0.003625811871560086;
    rectification_matrix_right.at<double>(0,2) = 0.007755443660172947;
    rectification_matrix_right.at<double>(1,0) = 0.003680398547259526;
    rectification_matrix_right.at<double>(1,1) = 0.9999684752771629;
    rectification_matrix_right.at<double>(1,2) = -0.007035845251224894;
    rectification_matrix_right.at<double>(2,0) = -0.007729688520722713;
    rectification_matrix_right.at<double>(2,1) = 0.007064130529506649;
    rectification_matrix_right.at<double>(2,2) = 0.999945173484644;

    projection_matrix_left.at<double>(0,0) = 435.2046959714599;
    projection_matrix_left.at<double>(0,2) = 367.4517211914062;
    projection_matrix_left.at<double>(1,1) = 435.2046959714599;
    projection_matrix_left.at<double>(1,2) = 252.2008514404297;

    projection_matrix_right.at<double>(0,0) = 435.2046959714599;
    projection_matrix_right.at<double>(0,2) = 367.4517211914062;
    projection_matrix_right.at<double>(0,3) = -47.90639384423901;
    projection_matrix_right.at<double>(1,1) = 435.2046959714599;
    projection_matrix_right.at<double>(1,2) = 252.2008514404297;
  } else {

    //ds use default best:
    throw std::runtime_error("not implemented");
  }

  //ds undistortion/rectification
  cv::Mat undistort_rectify_maps_left[2];
  cv::Mat undistort_rectify_maps_right[2];

  //ds compute undistorted and rectified mappings
  cv::initUndistortRectifyMap(camera_calibration_matrix_left,
                              distortion_coefficients_left,
                              rectification_matrix_left,
                              projection_matrix_left,
                              image_size,
                              CV_16SC2,
                              undistort_rectify_maps_left[0],
                              undistort_rectify_maps_left[1]);
  cv::initUndistortRectifyMap(camera_calibration_matrix_right,
                              distortion_coefficients_right,
                              rectification_matrix_right,
                              projection_matrix_right,
                              image_size,
                              CV_16SC2,
                              undistort_rectify_maps_right[0],
                              undistort_rectify_maps_right[1]);

  //ds check if rectification failed
  if (cv::norm(undistort_rectify_maps_left[0]) == 0 || cv::norm(undistort_rectify_maps_left[1]) == 0) {
    std::cerr << "ERROR: unable to undistort and rectify camera left" << std::endl;
    return 0;
  }
  if (cv::norm(undistort_rectify_maps_right[0]) == 0 || cv::norm(undistort_rectify_maps_right[1]) == 0) {
    std::cerr << "ERROR: unable to undistort and rectify camera right" << std::endl;
    return 0;
  }

  //ds get camera matrices back to eigen space
  Matrix3 camera_matrix_rectified(Matrix3::Zero());
  for(uint32_t row = 0; row < 3; ++row) {
    for(uint32_t col = 0; col < 3; ++col) {
      camera_matrix_rectified(row,col)  = projection_matrix_left.at<double>(row,col);
    }
  }

  //ds get right projection matrix to eigen space
  ProjectionMatrix projection_matrix_right_eigen(ProjectionMatrix::Zero());
  for(uint32_t row = 0; row < 3; ++row) {
    for(uint32_t col = 0; col < 4; ++col) {
      projection_matrix_right_eigen(row,col) = projection_matrix_right.at<double>(row,col);
    }
  }

  //ds consistency check
  if ((camera_matrix_rectified-projection_matrix_right_eigen.block<3,3>(0,0)).norm() != 0) {
    std::cerr << "ERROR: inconsistent projection matrices" << std::endl;
    return 0;
  }

  //ds compute offset for right camera in order to reconstruct projection matrix form txt_io message
  const Vector3 offset(camera_matrix_rectified.fullPivLu().solve(projection_matrix_right_eigen.block<3,1>(0,3)));
  TransformMatrix3D camera_left_to_right_rectified(TransformMatrix3D::Identity());
  camera_left_to_right_rectified.translation() = -offset;

  //ds open a message writer in the current directory
  MessageWriter txt_io_message_writer;
  txt_io_message_writer.open(outfile_name);

  //ds complete image vector
  std::vector<std::pair<uint64_t, std::string> > images = images_left;
  images.insert(images.end(), images_right.begin(), images_right.end());
  std::sort(images.begin(), images.end(), CustomComparator);

  //ds ground truth handle
  TransformMatrix3D imu_to_world(TransformMatrix3D::Identity());

  //ds start message dumping
  uint64_t sequence_number_image_left  = 0;
  uint64_t sequence_number_image_right = 0;
  for (uint32_t message_number = 0; message_number < images.size(); ++message_number) {

    //ds look for the ground truth pose
    if (option_convert_ground_truth) {
      uint32_t imprecision_offset = 0;
      while(imprecision_offset < 10) {

        //ds try positive offset
        try {
          imu_to_world = ground_truth.at(images[message_number].first+imprecision_offset).imu_to_world;
          break;
        } catch(std::out_of_range& /*exception*/) {

          //ds try negative offset
          try {
            imu_to_world = ground_truth.at(images[message_number].first-imprecision_offset).imu_to_world;
            break;
          } catch(std::out_of_range& /*exception*/) {
            ++imprecision_offset;
          }
        }
      }

      //ds check if failed
      if (10 == imprecision_offset) {
        std::cerr << "_";
        continue;
      }
    }

    //ds if left camera
    if (images[message_number].second.find("cam0") != std::string::npos) {

      //ds allocate pinhole image message
      srrg_core::PinholeImageMessage* txt_io_message = new srrg_core::PinholeImageMessage("/camera_left/image_raw", "camera_left");
      txt_io_message->setSeq(sequence_number_image_left);
      txt_io_message->setTimestamp(images[message_number].first/1e9);

      //ds set ground truth
      if (option_convert_ground_truth) {txt_io_message->setOdometry((imu_to_world*camera_left_to_imu).cast<float>());}

      //ds preprocess image
      cv::Mat image_raw = cv::imread(images[message_number].second, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image_undistorted_rectified;
      txt_io_message->setCameraMatrix(camera_matrix_rectified.cast<float>());
      txt_io_message->setOffset(TransformMatrix3D::Identity().cast<float>());

      //ds rectify image
      cv::remap(image_raw, image_undistorted_rectified, undistort_rectify_maps_left[0], undistort_rectify_maps_left[1], cv::INTER_LINEAR);

      //ds set image to message
      txt_io_message->setImage(image_undistorted_rectified);

      //ds save message to disk
      txt_io_message_writer.writeMessage(*txt_io_message);
      delete txt_io_message;

      std::cerr << "L";
      ++sequence_number_image_left;

    } else if (images[message_number].second.find("cam1") != std::string::npos) {

      //ds allocate pinhole image message
      srrg_core::PinholeImageMessage* txt_io_message = new PinholeImageMessage("/camera_right/image_raw", "camera_right");
      txt_io_message->setSeq(sequence_number_image_right);
      txt_io_message->setTimestamp(images[message_number].first/1e9);

      //ds set ground truth
      if (option_convert_ground_truth) {txt_io_message->setOdometry((imu_to_world*camera_right_to_imu).cast<float>());}

      //ds preprocess image
      cv::Mat image_raw = cv::imread(images[message_number].second, CV_LOAD_IMAGE_GRAYSCALE);
      cv::Mat image_undistorted_rectified;
      txt_io_message->setCameraMatrix(camera_matrix_rectified.cast<float>());
      txt_io_message->setOffset(camera_left_to_right_rectified.cast<float>());

      //ds rectify image
      cv::remap(image_raw, image_undistorted_rectified, undistort_rectify_maps_right[0], undistort_rectify_maps_right[1], cv::INTER_LINEAR);

      //ds set image to message
      txt_io_message->setImage(image_undistorted_rectified);

      //ds save message to disk
      txt_io_message_writer.writeMessage(*txt_io_message);
      delete txt_io_message;

      std::cerr << "R";
      ++sequence_number_image_right;

    } else {
      std::cerr << "ERROR: invalid message in the loop: " << images[message_number].second << std::endl;
      return 0;
    }
  }
  std::cerr << std::endl;
  std::cerr << "processed images  left: " << sequence_number_image_left << std::endl;
  std::cerr << "processed images right: " << sequence_number_image_right << std::endl;
  return 0;
}
