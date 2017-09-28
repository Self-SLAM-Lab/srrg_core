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
const std::string banner = "usage: ./visual_tracker <image_file_name_0> <image_file_name_1> [<image_file_name_x> .. -tracks <integer> -descriptor <brief/orb/brisk/freak>]";

//ds feature handling - nasty globals
cv::Ptr<cv::FastFeatureDetector> keypoint_detector;
cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
uint32_t target_number_of_tracks = 500;
std::string descriptor_type      = "brief";

//! @brief tracks features between two images
//! @param[in] image_from_ the preceding image in the motion
//! @param[in] image_to_ the current image in the motion
void trackExhaustive(const cv::Mat& image_from_, const cv::Mat& image_to_);

int32_t main (int32_t argc, char** argv) {

  //ds at least two images have to be provided
  if (argc < 3) {
    std::cerr << banner << std::endl;
    return EXIT_FAILURE;
  }

  //ds configuration
  std::vector<std::string> image_file_names(0);

  //ds parse arguments
  int32_t c = 1;
  while(c < argc) {
    if (!strcmp(argv[c], "-h")) {
      std::cerr << banner << std::endl;
      return EXIT_FAILURE;
    } else if (!strcmp(argv[c], "-tracks")) {
      c++;
      target_number_of_tracks = std::stoi(argv[c]);
    } else if (!strcmp(argv[c], "-descriptor")) {
      c++;
      descriptor_type = argv[c];
    } else {
      image_file_names.push_back(argv[c]);
    }
    c++;
  }

  //ds input validation
  if (image_file_names.empty()) {
    std::cerr << "ERROR: no images specified (provide at least 2: <image_file_name_0> <image_file_name_1>)" << std::endl;
    return EXIT_FAILURE;
  }

  //ds log configuration
  LOG_VARIABLE(image_file_names.size())
  std::cerr << "sequence:" << std::endl;
  for (std::string& image_file_name: image_file_names) {
    std::cerr << " " << image_file_name << std::endl;
  }
  LOG_VARIABLE(target_number_of_tracks)
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

  //ds process all images
  std::string image_file_name_previous = "";
  for (const std::string& image_file_name: image_file_names) {
    if (image_file_name_previous != "") {
      std::cerr << "--------------------------------------------------------------------------------------------" << std::endl;
      std::cerr << "computing tracks from: " << image_file_name_previous << std::endl;
      std::cerr << "                   to: " << image_file_name << std::endl;

      //ds load images from disk
      const cv::Mat image_from = cv::imread(image_file_name_previous);
      const cv::Mat image_to   = cv::imread(image_file_name);

      //ds if loading failed
      if (image_from.empty()) {
        std::cerr << "ERROR: unable to load image: '" << image_file_name_previous << "' (provide absolute path or check current folder)" << std::endl;
        return EXIT_FAILURE;
      }
      if (image_to.empty()) {
        std::cerr << "ERROR: unable to load image: '" << image_file_name_previous << "' (provide absolute path or check current folder)" << std::endl;
        return EXIT_FAILURE;
      }

      //ds compute tracks
      trackExhaustive(image_from, image_to);
    }
    image_file_name_previous = image_file_name;
  }
  return EXIT_SUCCESS;
}

void trackExhaustive(const cv::Mat& image_from_, const cv::Mat& image_to_) {

  //ds detect keypoints in both images
  std::vector<cv::KeyPoint> keypoints_from;
  std::vector<cv::KeyPoint> keypoints_to;
  std::chrono::time_point<std::chrono::system_clock> time_begin = std::chrono::system_clock::now();
  keypoint_detector->detect(image_from_, keypoints_from);
  keypoint_detector->detect(image_to_, keypoints_to);
  std::chrono::duration<double> duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "detected keypoints: " << keypoints_from.size() << ", " << keypoints_to.size()
            << " duration per keypoint (ms): " << duration.count()/(keypoints_from.size()+keypoints_to.size())*1e3 << std::endl;

  //ds compute descriptors for all keypoints
  cv::Mat descriptors_from;
  cv::Mat descriptors_to;
  time_begin = std::chrono::system_clock::now();
  descriptor_extractor->compute(image_from_, keypoints_from, descriptors_from);
  descriptor_extractor->compute(image_to_, keypoints_to, descriptors_to);
  duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "computed descriptors: " << descriptors_from.rows << ", " << descriptors_to.rows
            << " duration per descriptor (ms): " << duration.count()/(descriptors_from.rows+descriptors_to.rows)*1e3 << std::endl;

  //ds find best matches (exhaustive)
  std::vector<cv::DMatch> descriptor_matches;
  descriptor_matcher->match(descriptors_from, descriptors_to, descriptor_matches);
  std::cerr << "found matches: " << descriptor_matches.size() << " displaying best: " << target_number_of_tracks << std::endl;

  //ds sort matches by ascending matching distance
  std::sort(descriptor_matches.begin(), descriptor_matches.end(), [](const cv::DMatch& a_, const cv::DMatch& b_){return a_.distance < b_.distance;});

  //ds display matches (TODO check color conversion)
  cv::Mat image_display_combined;
  cv::vconcat(image_from_, image_to_, image_display_combined);
  const cv::Point2f offset(0, image_from_.rows);

  //ds draw a selection of the best tracks
  for (uint32_t index = 0; index < target_number_of_tracks; ++index) {
    if (index < descriptor_matches.size()) {
      const cv::KeyPoint& keypoint_from = keypoints_from[descriptor_matches[index].queryIdx];
      const cv::KeyPoint& keypoint_to   = keypoints_to[descriptor_matches[index].trainIdx];

      //ds draw keypoints: from in red, to in blue
      cv::circle(image_display_combined, keypoint_from.pt, 3, cv::Scalar(0, 0, 255), 1);
      cv::circle(image_display_combined, keypoint_to.pt+offset, 3, cv::Scalar(255, 0, 0), 1);

      //ds draw connecting tracks
      cv::line(image_display_combined, keypoint_from.pt, keypoint_to.pt+offset, cv::Scalar(0, 255, 0), 1);
    }
  }

  //ds log current highest matching distance
  if (target_number_of_tracks < descriptor_matches.size()) {
    std::cerr << "current maximum matching distance: " << descriptor_matches[target_number_of_tracks].distance << std::endl;
  } else {
    std::cerr << "current maximum matching distance: " << descriptor_matches.back().distance << std::endl;
  }

  //ds display image (blocking)
  cv::imshow("tracks (RED: from, BLUE: to)", image_display_combined);
  cv::waitKey(0);
}
