#include <iostream>
#include <chrono>
#include <ctime>
#include <fstream>

//ds resolve opencv includes
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION == 2
#elif CV_MAJOR_VERSION == 3
  #ifdef SRRG_CORE_HAS_OPENCV_CONTRIB
    #include <opencv2/xfeatures2d.hpp>
  #endif
#else
  #error OpenCV version not supported
#endif

//ds easy logging macro
#define LOG_VARIABLE(VARIABLE_) \
  std::cerr << #VARIABLE_ << ": " << VARIABLE_ << std::endl;

//ds info banner
const std::string banner = "usage: ./triangulator_stereo -l <image_file_name> -r <image_file_name> [-matches <integer> -descriptor <surf/brief/orb/brisk/freak>]";

//ds feature handling - nasty globals
cv::Ptr<cv::FastFeatureDetector> keypoint_detector;
cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
uint32_t target_number_of_matches = 1000;
std::string descriptor_type       = "surf";

//ds stereo triangulation
float f_u = 0;
float f_v = 0;
float c_u = 0;
float c_v = 0;
float b_u = 0;

//! @brief loads camera calibration data from a KITTI calib.txt file
//! @param[in] file_name_calibration_ calibration file
void loadStereoCameraCalibration(const std::string& file_name_calibration_);

//! @brief computes stereo feature matches between two images
//! @param[in] image_left_ the left camera image
//! @param[in] image_right_ the right camera image
void matchExhaustive(const cv::Mat& image_left_,
                     const cv::Mat& image_right_);

//! @brief triangulates a point in camera frame
//! @param[in] point_in_image_left_
//! @param[in] point_in_image_right_
//! @param[in] f_u_ horizontal focal length
//! @param[in] f_v_ vertical focal length (usually == f_u_)
//! @param[in] c_u_ horizontal principal point coordinate
//! @param[in] c_v_ vertical principal point coordinate
//! @param[in] b_u_ stereo camera baseline
const cv::Point3f getPointInCamera(const cv::Point2f& point_in_image_left_,
                                   const cv::Point2f& point_in_image_right_,
                                   const float& f_u_,
                                   const float& f_v_,
                                   const float& c_u_,
                                   const float& c_v_,
                                   const float& b_u_);

int32_t main (int32_t argc, char** argv) {

  //ds at least two images have to be provided
  if (argc < 3) {
    std::cerr << banner << std::endl;
    return 0;
  }

  //ds configuration
  std::string file_name_image_left  = "";
  std::string file_name_image_right = "";
  std::string file_name_calibration = "";

  //ds parse arguments
  int32_t c = 1;
  while(c < argc) {
    if (!strcmp(argv[c], "-h")) {
      std::cerr << banner << std::endl;
      return 0;
    } else if (!strcmp(argv[c], "-l")) {
      c++;
      file_name_image_left = argv[c];
    } else if (!strcmp(argv[c], "-r")) {
      c++;
      file_name_image_right = argv[c];
    } else if (!strcmp(argv[c], "-matches")) {
      c++;
      target_number_of_matches = std::stoi(argv[c]);
    } else if (!strcmp(argv[c], "-descriptor")) {
      c++;
      descriptor_type = argv[c];
    } else if (!strcmp(argv[c], "-calibration")) {
      c++;
      file_name_calibration = argv[c];
    }
    c++;
  }

  //ds input validation
  if (file_name_image_left.empty()) {
    std::cerr << "ERROR: left image not specified (use -l <image_file_name>)" << std::endl;
    return 0;
  }
  if (file_name_image_right.empty()) {
    std::cerr << "ERROR: right image not specified (use -r <image_file_name>)" << std::endl;
    return 0;
  }

  //ds load camera calibration if provided (KITTI only) - required for triangulation
  loadStereoCameraCalibration(file_name_calibration);

  //ds log configuration
  LOG_VARIABLE(file_name_image_left)
  LOG_VARIABLE(file_name_image_right)
  LOG_VARIABLE(target_number_of_matches)
  LOG_VARIABLE(descriptor_type)

  //ds allocate feature handlers
#if CV_MAJOR_VERSION == 2
  keypoint_detector = new cv::FastFeatureDetector();
  if (descriptor_type == "brief") {
    descriptor_extractor = new cv::BriefDescriptorExtractor(32);
  } else if (descriptor_type == "orb") {
    descriptor_extractor = new cv::ORB();
  } else if (descriptor_type == "brisk") {
    descriptor_extractor = new cv::BRISK();
  } else if (descriptor_type == "freak") {
    descriptor_extractor = new cv::FREAK();
  } else {
    std::cerr << "ERROR: invalid descriptor_type: " << descriptor_type << std::endl;
    return 0;
  }
  descriptor_matcher = new cv::BFMatcher(cv::NORM_HAMMING);
#elif CV_MAJOR_VERSION == 3
  keypoint_detector = cv::FastFeatureDetector::create();
  if (descriptor_type == "surf") {
#ifdef SRRG_CORE_HAS_OPENCV_CONTRIB
    descriptor_extractor = cv::xfeatures2d::SURF::create();
#else
    std::cerr << "ERROR: descriptor_type not available: " << descriptor_type << std::endl;
    return 0;
#endif
  } else if (descriptor_type == "brief") {
#ifdef SRRG_CORE_HAS_OPENCV_CONTRIB
    descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
#else
    std::cerr << "ERROR: descriptor_type not available: " << descriptor_type << std::endl;
    return 0;
#endif
  } else if (descriptor_type == "orb") {
    descriptor_extractor = cv::ORB::create();
  } else if (descriptor_type == "brisk") {
    descriptor_extractor = cv::BRISK::create();
  } else if (descriptor_type == "freak") {
#ifdef SRRG_CORE_HAS_OPENCV_CONTRIB
    descriptor_extractor = cv::xfeatures2d::FREAK::create();
#else
    std::cerr << "ERROR: descriptor_type not available: " << descriptor_type << std::endl;
    return 0;
#endif
  } else if (descriptor_type == "akaze") {
    descriptor_extractor = cv::AKAZE::create();
  } else {
    std::cerr << "ERROR: invalid descriptor_type: " << descriptor_type << std::endl;
    return 0;
  }
  if (descriptor_type == "surf") {
    descriptor_matcher = cv::BFMatcher::create(cv::NORM_L2);
  } else {
    descriptor_matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
  }
#endif

  //ds load images from disk
  const cv::Mat image_left  = cv::imread(file_name_image_left);
  const cv::Mat image_right = cv::imread(file_name_image_right);

  //ds if loading failed
  if (image_left.empty()) {
    std::cerr << "ERROR: unable to load image: '" << file_name_image_left << "' (provide absolute path or check current folder)" << std::endl;
    return 0;
  }
  if (image_right.empty()) {
    std::cerr << "ERROR: unable to load image: '" << file_name_image_right << "' (provide absolute path or check current folder)" << std::endl;
    return 0;
  }

  //ds compute stereo correspondences
  matchExhaustive(image_left, image_right);
  return 0;
}

void loadStereoCameraCalibration(const std::string& file_name_calibration_) {
  if (!file_name_calibration_.empty()) {
    std::ifstream stream_calibration(file_name_calibration_, std::ifstream::in);
    std::string line_buffer("");
    std::getline(stream_calibration, line_buffer);
    if (line_buffer.empty()) {
      std::cerr << "ERROR: invalid camera calibration file provided: " << file_name_calibration_ << std::endl;
      exit(0);
    }
    std::istringstream stream(line_buffer);

    //ds parse in fixed order
    std::string filler("");
    stream >> filler;
    stream >> f_u;
    stream >> filler;
    stream >> c_u;
    stream >> filler;
    stream >> filler;
    stream >> f_v;
    stream >> c_v;

    //ds check for invalid result (simple)
    if (f_u == 0 || f_v == 0 || c_u == 0 || c_v == 0) {
      std::cerr << "ERROR: invalid stereo camera calibration file provided: " << file_name_calibration_ << std::endl;
      exit(0);
    }

    //ds scan second line for baseline
    std::getline(stream_calibration, line_buffer);
    if (line_buffer.empty()) {
      std::cerr << "ERROR: invalid stereo camera calibration file provided: " << file_name_calibration_ << std::endl;
      exit(0);
    }
    stream = std::istringstream(line_buffer);

    //ds parse in fixed order
    stream >> filler;
    stream >> filler;
    stream >> filler;
    stream >> filler;
    stream >> b_u;
    stream_calibration.close();


    //ds check for invalid result (simple)
    if (f_u == 0 || f_v == 0 || c_u == 0 || c_v == 0 || b_u == 0) {
      std::cerr << "ERROR: invalid stereo camera calibration file provided: " << file_name_calibration_ << std::endl;
      exit(0);
    }

    //ds info
    std::cerr << "loaded stereo camera calibration matrix with parameters: " << std::endl;
    LOG_VARIABLE(f_u)
    LOG_VARIABLE(f_v)
    LOG_VARIABLE(c_u)
    LOG_VARIABLE(c_v)
    LOG_VARIABLE(b_u)
  }
}

void matchExhaustive(const cv::Mat& image_left_, const cv::Mat& image_right_) {

  //ds detect keypoints in both images
  std::vector<cv::KeyPoint> keypoints_left;
  std::vector<cv::KeyPoint> keypoints_right;
  std::chrono::time_point<std::chrono::system_clock> time_begin = std::chrono::system_clock::now();
  keypoint_detector->detect(image_left_, keypoints_left);
  keypoint_detector->detect(image_right_, keypoints_right);
  std::chrono::duration<float> duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "detected keypoints: " << keypoints_left.size() << ", " << keypoints_right.size()
            << " duration per keypoint (ms): " << duration.count()/(keypoints_left.size()+keypoints_right.size())*1e3
            << " total (ms): " << duration.count()*1e3 << std::endl;

  //ds compute descriptors for all keypoints
  cv::Mat descriptors_left;
  cv::Mat descriptors_right;
  time_begin = std::chrono::system_clock::now();
  descriptor_extractor->compute(image_left_, keypoints_left, descriptors_left);
  descriptor_extractor->compute(image_right_, keypoints_right, descriptors_right);
  duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "computed descriptors: " << descriptors_left.rows << ", " << descriptors_right.rows
            << " duration per descriptor (ms): " << duration.count()/(descriptors_left.rows+descriptors_right.rows)*1e3
            << " total (ms): " << duration.count()*1e3 << std::endl;

  //ds find best matches (exhaustive)
  std::vector<cv::DMatch> descriptor_matches;
  descriptor_matcher->match(descriptors_left, descriptors_right, descriptor_matches);
  std::cerr << "found matches: " << descriptor_matches.size() << " displaying best: " << target_number_of_matches << std::endl;

  //ds sort matches by ascending matching distance
  std::sort(descriptor_matches.begin(), descriptor_matches.end(), [](const cv::DMatch& a_, const cv::DMatch& b_){return a_.distance < b_.distance;});

  //ds display matches
  cv::Mat image_display_combined;
  cv::vconcat(image_left_, image_right_, image_display_combined);
  const cv::Point2f offset(0, image_left_.rows);

  //ds display depths
  cv::Mat image_display_depth(image_left_.rows, image_left_.cols, CV_8UC3);
  image_display_depth.setTo(0);

  //ds accumulated row distance (with perfect calibration and rectification without distortion -> 0)
  float accumulated_row_distance = 0;
  uint32_t number_of_zero_vertical_distances   = 0;
  uint32_t number_of_one_vertical_distances    = 0;
  uint32_t number_of_two_vertical_distances    = 0;
  uint32_t number_of_higher_vertical_distances = 0;
  uint32_t number_of_triangulated_points = 0;
  uint32_t number_of_infinity_points     = 0;

  //ds draw a selection of the best tracks
  for (uint32_t index = 0; index < target_number_of_matches; ++index) {
    if (index < descriptor_matches.size()) {

      //ds generate a random color to indicate the match
      cv::Scalar color(rand() % 255,rand() % 255,rand() % 255);

      //ds grab keypoints
      const cv::KeyPoint& keypoint_left = keypoints_left[descriptor_matches[index].queryIdx];
      const cv::KeyPoint& keypoint_right   = keypoints_right[descriptor_matches[index].trainIdx];
      const float row_distance = std::fabs(keypoint_left.pt.y-keypoint_right.pt.y);
      accumulated_row_distance += row_distance;
      if (row_distance == 0) {
        ++number_of_zero_vertical_distances;
      } else if (row_distance == 1) {
        ++number_of_one_vertical_distances;
      } else if (row_distance == 2) {
        ++number_of_two_vertical_distances;
      } else {
        ++number_of_higher_vertical_distances;
      }

      //ds draw keypoints: from in red, to in blue
      cv::circle(image_display_combined, keypoint_left.pt, 4, color, 1);
      cv::circle(image_display_combined, keypoint_right.pt+offset, 4, color, 1);

      try {

        //ds compute 3d point in camera
        const cv::Point3f point_in_camera_left(getPointInCamera(keypoint_left.pt, keypoint_right.pt, f_u, f_v, c_u, c_v, b_u));
        float depth_meters = point_in_camera_left.z;
        ++number_of_triangulated_points;

        //ds determine color intensity
        const float intensity = std::max((1-depth_meters/25)*255, 50.0f);

        //ds if set
        color = cv::Scalar(0, 0, intensity);
        cv::circle(image_display_depth, keypoint_left.pt, 1, color, -1);
      } catch (const std::runtime_error& /*ex*/) {

        //ds point has zero disparity
        ++number_of_infinity_points;
      }
    }
  }

  //ds log stats
  if (target_number_of_matches < descriptor_matches.size()) {
    const float average_row_distance = accumulated_row_distance/target_number_of_matches;
    std::cerr << "maximum matching distance: " << descriptor_matches[target_number_of_matches].distance << std::endl;
    std::cerr << "average row distance: " << average_row_distance << std::endl;
  } else {
    const float average_row_distance = accumulated_row_distance/descriptor_matches.size();
    std::cerr << "maximum matching distance: " << descriptor_matches.back().distance << std::endl;
    std::cerr << "average row distance: " << average_row_distance << std::endl;
  }
  LOG_VARIABLE(number_of_zero_vertical_distances)
  LOG_VARIABLE(number_of_one_vertical_distances)
  LOG_VARIABLE(number_of_two_vertical_distances)
  LOG_VARIABLE(number_of_higher_vertical_distances)
  LOG_VARIABLE(number_of_triangulated_points)
  LOG_VARIABLE(number_of_infinity_points)

  //ds display image (blocking)
  cv::imshow("stereo matches (matches have same color)", image_display_combined);
  cv::imshow("depth (bright red: close, dark red: far, black: not set)", image_display_depth);
  std::cerr << "press [any key] to terminate (in the OpenCV windows)" << std::endl;
  cv::waitKey(0);
  cv::destroyAllWindows();
}

const cv::Point3f getPointInCamera(const cv::Point2f& point_in_image_left_,
                                   const cv::Point2f& point_in_image_right_,
                                   const float& f_u_,
                                   const float& f_v_,
                                   const float& c_u_,
                                   const float& c_v_,
                                   const float& b_u_) {

  //ds check for insufficient disparity (point at infinity
  if (point_in_image_left_.x-point_in_image_right_.x <  1) {
    throw std::runtime_error("zero disparity");
  }

  //ds compute depth from disparity
  const float depth_meters = b_u_/(point_in_image_right_.x-point_in_image_left_.x);

  //ds compute full point
  const float depth_meters_per_pixel = depth_meters/f_u_;
  return cv::Point3f(depth_meters_per_pixel*(point_in_image_left_.x-c_u_),
                     depth_meters_per_pixel*(point_in_image_left_.y-c_v_),
                     depth_meters);
}
