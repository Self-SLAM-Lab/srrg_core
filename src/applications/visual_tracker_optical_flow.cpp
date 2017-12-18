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
  std::cerr << #VARIABLE_ << ": " << VARIABLE_ << std::endl

//ds info banner
const std::string banner = "usage: ./visual_tracker <image_file_name_0> <image_file_name_1> [<image_file_name_x> .. -norm-threshold <float> -descriptor <brief/orb/brisk/freak>]";

//ds feature handling - nasty globals
cv::Ptr<cv::FastFeatureDetector> keypoint_detector;
cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
  float norm_threshold = 25;
std::string descriptor_type      = "brief";

//ds container holding spatial and appearance information (used in findStereoKeypoints)
struct KeypointWithDescriptor {
  cv::KeyPoint keypoint;
  cv::Mat descriptor;
  bool available;
};

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
    } else if (!strcmp(argv[c], "-norm-threshold")) {
      c++;
      norm_threshold = std::stof(argv[c]);
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
  LOG_VARIABLE(image_file_names.size());
  std::cerr << "sequence:" << std::endl;
  for (std::string& image_file_name: image_file_names) {
    std::cerr << " " << image_file_name << std::endl;
  }
  LOG_VARIABLE(norm_threshold);
  LOG_VARIABLE(descriptor_type);

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
    //image_file_name_previous = image_file_name;
    image_file_name_previous = image_file_names[0];
  }
  return EXIT_SUCCESS;
}

void trackExhaustive(const cv::Mat& image_from_, const cv::Mat& image_to_) {

  //wv detect keypoints in the first image
  std::vector<cv::KeyPoint> keypoints_from;
  float keypoint_size=0;
  
  std::chrono::time_point<std::chrono::system_clock> time_begin = std::chrono::system_clock::now();
  keypoint_detector->detect(image_from_, keypoints_from);
  keypoint_size = keypoints_from[0].size;
  
  std::chrono::duration<double> duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "detected keypoints: " << keypoints_from.size() << ", "
            << " duration per keypoint (ms): " << duration.count()/(keypoints_from.size())*1e3 << std::endl;

  
  //ws compute descriptors for all keypoints_from
  cv::Mat descriptors_from;
  time_begin = std::chrono::system_clock::now();
  descriptor_extractor->compute(image_from_, keypoints_from, descriptors_from);

  duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "computed descriptors_from: " << descriptors_from.rows << ", "
            << " duration per descriptor (ms): " << duration.count()/(descriptors_from.rows)*1e3 << std::endl;

	    
  //wv get points from keypoints_from
  std::vector<cv::Point2f> points_from;
  int num_points = keypoints_from.size();
  for (int i=0; i< num_points; i++) {
    points_from.push_back(keypoints_from[i].pt);  
  }
  
  
  //wv get points_to from points_from with optical flow
  std::vector<cv::Point2f> points_to;
  std::vector<uchar> status;
  std::vector<float> err;
  cv::Size winSize(31,31);
  int max_lvl = 3;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
  int flags = 0;
  double migEighThreshold = 0.001;
  
  cv::calcOpticalFlowPyrLK(image_from_, image_to_, points_from, points_to, status, err, winSize, max_lvl, termcrit, flags, migEighThreshold); 
  
  
  //ws extract a descriptor for each point obtained from optical flow
  time_begin = std::chrono::system_clock::now();
  uint number_of_tracks_with_descriptors = 0;
  std::vector<KeypointWithDescriptor> keypoints_with_descriptors_to(num_points);
  
  for( int i = 0; i < num_points; i++){
    std::vector<cv::KeyPoint> keypoints_buffer(1, cv::KeyPoint(points_to[i], keypoint_size));
    
    cv::Mat descriptor;
    descriptor_extractor->compute(image_to_, keypoints_buffer, descriptor);
    
    //ws does not exits a descriptor for the current keypoint
    if (keypoints_buffer.size() == 0) {
      keypoints_with_descriptors_to[i].available = false;
    } else {
    //ws exits a descriptor for the current keypoint
      keypoints_with_descriptors_to[i].available = true;
      keypoints_with_descriptors_to[i].keypoint = keypoints_buffer[0];
      keypoints_with_descriptors_to[i].descriptor = descriptor;
      ++number_of_tracks_with_descriptors;
    }
  }
  
  duration = std::chrono::system_clock::now()-time_begin;
  std::cerr << "computed descriptors_to: " << number_of_tracks_with_descriptors << ", "
            << " duration per descriptor (ms): " << duration.count()/number_of_tracks_with_descriptors*1e3 << std::endl;
  
	    
  //ws display matches between the two images	    
  cv::Mat image_display_combined;
  cv::vconcat(image_from_, image_to_, image_display_combined);
  const cv::Point2f offset(0, image_from_.rows);
  
  for (int i=0; i < num_points; i++) {    
      if (keypoints_with_descriptors_to[i].available && status[i]) {
	
	double norm = cv::norm(descriptors_from.row(i), keypoints_with_descriptors_to[i].descriptor, cv::NORM_HAMMING);
	if( norm <= norm_threshold){ 
	  cv::circle(image_display_combined, keypoints_from[i].pt, 3, cv::Scalar(0, 0, 255), 1);
	  cv::circle(image_display_combined, keypoints_with_descriptors_to[i].keypoint.pt + offset, 3, cv::Scalar(255, 0, 0), 1);
	  
	  cv::line(image_display_combined, keypoints_from[i].pt, keypoints_with_descriptors_to[i].keypoint.pt + offset, cv::Scalar(0, 255, 0), 1);
	}
      }   
  }
  
  cv::imshow("tracks (RED: from, BLUE: to)4", image_display_combined);
  cv::waitKey(0);
}
