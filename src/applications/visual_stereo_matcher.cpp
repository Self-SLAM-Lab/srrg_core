#include <iostream>
#include <chrono>
#include <ctime>

//ds resolve opencv includes
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION == 2
  //ds no specifics
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/xfeatures2d.hpp>
#else
  #error OpenCV version not supported
#endif

//ds easy logging macro
#define LOG_VARIABLE(VARIABLE_) \
  std::cerr << #VARIABLE_ << ": " << VARIABLE_ << std::endl;

//ds info banner
const std::string banner = "usage: ./visual_stereo_matcher -l <image_file_name> -r <image_file_name> [-matches <integer> -descriptor <brief/orb/brisk/freak>]";

//ds feature handling - nasty globals
cv::Ptr<cv::FastFeatureDetector> keypoint_detector;
cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
uint32_t target_number_of_matches = 500;
std::string descriptor_type       = "brief";

//! @brief computes stereo feature matches between two images
//! @param[in] image_left_ the left camera image
//! @param[in] image_right_ the right camera image
void matchExhaustive(const cv::Mat& image_left_, const cv::Mat& image_right_);

int32_t main (int32_t argc, char** argv) {

  //ds at least two images have to be provided
  if (argc < 3) {
    std::cerr << banner << std::endl;
    return EXIT_FAILURE;
  }

  //ds configuration
  std::string file_name_image_left  = "";
  std::string file_name_image_right = "";

  //ds parse arguments
  int32_t c = 1;
  while(c < argc) {
    if (!strcmp(argv[c], "-h")) {
      std::cerr << banner << std::endl;
      return EXIT_FAILURE;
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
    }
    c++;
  }

  //ds input validation
  if (file_name_image_left.empty()) {
    std::cerr << "ERROR: left image not specified (use -l <image_file_name>)" << std::endl;
    return EXIT_FAILURE;
  }
  if (file_name_image_right.empty()) {
    std::cerr << "ERROR: right image not specified (use -r <image_file_name>)" << std::endl;
    return EXIT_FAILURE;
  }

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
    return EXIT_FAILURE;
  }
  descriptor_matcher = new cv::BFMatcher(cv::NORM_HAMMING);
#elif CV_MAJOR_VERSION == 3
  keypoint_detector = cv::FastFeatureDetector::create();
  if (descriptor_type == "brief") {
    descriptor_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
  } else if (descriptor_type == "orb") {
    descriptor_extractor = cv::ORB::create();
  } else if (descriptor_type == "brisk") {
    descriptor_extractor = cv::BRISK::create();
  } else if (descriptor_type == "freak") {
    descriptor_extractor = cv::xfeatures2d::FREAK::create();
  } else if (descriptor_type == "akaze") {
    descriptor_extractor = cv::AKAZE::create();
  } else {
    std::cerr << "ERROR: invalid descriptor_type: " << descriptor_type << std::endl;
    return EXIT_FAILURE;
  }
  descriptor_matcher   = cv::BFMatcher::create(cv::NORM_HAMMING);
#endif

  //ds load images from disk
  const cv::Mat image_left  = cv::imread(file_name_image_left);
  const cv::Mat image_right = cv::imread(file_name_image_right);

  //ds if loading failed
  if (image_left.empty()) {
    std::cerr << "ERROR: unable to load image: '" << file_name_image_left << "' (provide absolute path or check current folder)" << std::endl;
    return EXIT_FAILURE;
  }
  if (image_right.empty()) {
    std::cerr << "ERROR: unable to load image: '" << file_name_image_right << "' (provide absolute path or check current folder)" << std::endl;
    return EXIT_FAILURE;
  }

  //ds compute stereo correspondences
  matchExhaustive(image_left, image_right);
  return EXIT_SUCCESS;
}

void matchExhaustive(const cv::Mat& image_left_, const cv::Mat& image_right_) {

  //ds detect keypoints in both images
  std::vector<cv::KeyPoint> keypoints_from;
  std::vector<cv::KeyPoint> keypoints_to;
  std::chrono::time_point<std::chrono::system_clock> time_begin = std::chrono::system_clock::now();
  keypoint_detector->detect(image_left_, keypoints_from);
  keypoint_detector->detect(image_right_, keypoints_to);
  std::chrono::duration<double> duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "detected keypoints: " << keypoints_from.size() << ", " << keypoints_to.size()
            << " duration per keypoint (ms): " << duration.count()/(keypoints_from.size()+keypoints_to.size())*1e3 << std::endl;

  //ds compute descriptors for all keypoints
  cv::Mat descriptors_from;
  cv::Mat descriptors_to;
  time_begin = std::chrono::system_clock::now();
  descriptor_extractor->compute(image_left_, keypoints_from, descriptors_from);
  descriptor_extractor->compute(image_right_, keypoints_to, descriptors_to);
  duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "computed descriptors: " << descriptors_from.rows << ", " << descriptors_to.rows
            << " duration per descriptor (ms): " << duration.count()/(descriptors_from.rows+descriptors_to.rows)*1e3 << std::endl;

  //ds find best matches (exhaustive)
  std::vector<cv::DMatch> descriptor_matches;
  descriptor_matcher->match(descriptors_from, descriptors_to, descriptor_matches);
  std::cerr << "found matches: " << descriptor_matches.size() << " displaying best: " << target_number_of_matches << std::endl;

  //ds sort matches by ascending matching distance
  std::sort(descriptor_matches.begin(), descriptor_matches.end(), [](const cv::DMatch& a_, const cv::DMatch& b_){return a_.distance < b_.distance;});

  //ds display matches (TODO check color conversion)
  cv::Mat image_display_combined;
  cv::hconcat(image_left_, image_right_, image_display_combined);
  const cv::Point2f offset(image_left_.cols, 0);

  //ds accumulated row distance (with perfect calibration and rectification without distortion -> 0)
  double accumulated_row_distance   = 0;
  uint32_t number_of_zero_distances = 0;
  uint32_t number_of_one_distances  = 0;
  uint32_t number_of_two_distances  = 0;

  //ds draw a selection of the best tracks
  for (uint32_t index = 0; index < target_number_of_matches; ++index) {
    if (index < descriptor_matches.size()) {

      //ds generate a random color to indicate the match
      cv::Scalar color(rand() % 255,rand() % 255,rand() % 255);

      //ds grab keypoints
      const cv::KeyPoint& keypoint_from = keypoints_from[descriptor_matches[index].queryIdx];
      const cv::KeyPoint& keypoint_to   = keypoints_to[descriptor_matches[index].trainIdx];
      const double row_distance = std::fabs(keypoint_from.pt.y-keypoint_to.pt.y);
      accumulated_row_distance += row_distance;
      if (row_distance == 0) {
        ++number_of_zero_distances;
      } else if (row_distance == 1) {
        ++number_of_one_distances;
      } else if (row_distance == 2) {
        ++number_of_two_distances;
      }

      //ds draw keypoints: from in red, to in blue
      cv::circle(image_display_combined, keypoint_from.pt, 4, color, 1);
      cv::circle(image_display_combined, keypoint_to.pt+offset, 4, color, 1);
    }
  }

  //ds log stats
  if (target_number_of_matches < descriptor_matches.size()) {
    const double average_row_distance = accumulated_row_distance/target_number_of_matches;
    std::cerr << "maximum matching distance: " << descriptor_matches[target_number_of_matches].distance << std::endl;
    std::cerr << "average row distance: " << average_row_distance << std::endl;
  } else {
    const double average_row_distance = accumulated_row_distance/descriptor_matches.size();
    std::cerr << "maximum matching distance: " << descriptor_matches.back().distance << std::endl;
    std::cerr << "average row distance: " << average_row_distance << std::endl;
  }
  LOG_VARIABLE(number_of_zero_distances)
  LOG_VARIABLE(number_of_one_distances)
  LOG_VARIABLE(number_of_two_distances)

  //ds display image (blocking)
  cv::imshow("stereo matches (correspondences in same color)", image_display_combined);
  std::cerr << "press [any key] to terminate" << std::endl;
  cv::waitKey(0);
}
