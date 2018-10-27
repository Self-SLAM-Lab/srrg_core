#include <iostream>
#include <chrono>
#include <ctime>
#include <fstream>

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

//ds message loading
#include "srrg_messages/message_reader.h"
#include "srrg_messages/pinhole_image_message.h"
#include "srrg_messages/message_timestamp_synchronizer.h"

//ds easy logging macro
#define LOG_VARIABLE(VARIABLE_) \
  std::cerr << #VARIABLE_ << ": " << VARIABLE_ << std::endl

//ds info banner
const std::string banner = "usage: ./triangulator_mono -messages <kitti-srrg-messages>";

//ds feature handling - nasty globals
cv::Ptr<cv::FastFeatureDetector> keypoint_detector;
cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;

//ds container holding spatial and appearance information
struct KeypointWithDescriptor {
  KeypointWithDescriptor(): identifier(0) {}
  KeypointWithDescriptor(const cv::KeyPoint& keypoint_,
                         const cv::Mat& descriptor_): keypoint(keypoint_),
                                                      descriptor(descriptor_),
                                                      identifier(number_of_points) {++number_of_points;}

  cv::KeyPoint keypoint;
  cv::Mat descriptor;
  uint64_t identifier;
  static uint64_t number_of_points;
};
uint64_t KeypointWithDescriptor::number_of_points = 0;

//ds configuration
std::string descriptor_type        = "brief";
std::string message_file           = "";
std::string topic_camera_left      = "/camera_left/image_raw";
std::string topic_camera_right     = "/camera_right/image_raw";
uint32_t fast_detector_threshold   = 100;
uint32_t maximum_matching_distance = 25;

uint32_t minimum_track_length_for_triangulation = 10;
std::map<uint32_t, Eigen::Vector3f> stereo_points;

//ds finds matches between two point sets (can be used for stereo matching as well as tracking)
std::vector<std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>> getMatches(const std::vector<KeypointWithDescriptor*>& points_a_,
                                                                                    const std::vector<KeypointWithDescriptor*>& points_b_);

//ds triangulation function
Eigen::Vector3f getPointTriangulatedLS(const Eigen::Matrix3f& camera_calibration_matrix_,
                                              const std::vector<const KeypointWithDescriptor*>& track_,
                                              const std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>& world_to_cameras_left_);

Eigen::Vector3f getImagePointNormalized(const cv::Point2f& point_, const Eigen::Matrix3f& camera_calibration_matrix_);

//ds retrieves camera points for two corresponding image points
std::pair<Eigen::Vector3f, Eigen::Vector3f> getCameraPoints(const cv::Point2f image_point_previous_,
                                                            const cv::Point2f image_point_current_,
                                                            const Eigen::Isometry3f& camera_previous_to_current_,
                                                            const Eigen::Matrix3f& camera_calibration_matrix_);

int32_t main (int32_t argc, char** argv) {

  //ds at least message file must be provided
  if (argc < 2) {
    std::cerr << banner << std::endl;
    return EXIT_FAILURE;
  }

  //ds parse arguments
  int32_t c = 1;
  while(c < argc) {
    if (!strcmp(argv[c], "-h")) {
      std::cerr << banner << std::endl;
      return EXIT_FAILURE;
    } else if (!strcmp(argv[c], "-descriptor")) {
      c++;
      descriptor_type = argv[c];
    } else if (!strcmp(argv[c], "-messages")) {
      c++;
      message_file = argv[c];
    }
    c++;
  }

  //ds input validation
  if (message_file == "") {
    std::cerr << "ERROR: no input messages specified (use -messages <kitti-srrg-messages>)" << std::endl;
    return EXIT_FAILURE;
  }

  //ds log configuration
  LOG_VARIABLE(descriptor_type);
  LOG_VARIABLE(maximum_matching_distance);
  LOG_VARIABLE(message_file);

  //ds allocate feature handlers
#if CV_MAJOR_VERSION == 2
  keypoint_detector = new cv::FastFeatureDetector(fast_detector_threshold);
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

  //ds message playback
  srrg_core::MessageReader message_reader;
  srrg_core::MessageTimestampSynchronizer synchronizer;
  synchronizer.setTimeInterval(0.001);
  std::vector<std::string> camera_topics_synchronized(0);
  camera_topics_synchronized.push_back(topic_camera_left);
  camera_topics_synchronized.push_back(topic_camera_right);
  synchronizer.setTopics(camera_topics_synchronized);

  //ds camera configuration (the right camera is only used for validation purposes)
  Eigen::Matrix3f camera_calibration_matrix(Eigen::Matrix3f::Zero()); //ds K
  Eigen::Matrix<float, 3, 4> projection_matrix_left(Eigen::Matrix<float, 3, 4>::Zero()); //ds P
  Eigen::Matrix<float, 3, 4> projection_matrix_right(Eigen::Matrix<float, 3, 4>::Zero()); //ds P

  //ds quickly read the first messages to buffer camera info
  message_reader.open(message_file);
  srrg_core::BaseMessage* message = 0;
  while ((message = message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);
    if (sensor_message) {

      //ds check for the two set topics to set the camera objects
      if (sensor_message->topic() == topic_camera_left) {
        srrg_core::PinholeImageMessage* message_image_left = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_message);
        camera_calibration_matrix                      = message_image_left->cameraMatrix();
        projection_matrix_left.block<3,3>(0, 0) = camera_calibration_matrix;
      } else if (sensor_message->topic() == topic_camera_right) {
        srrg_core::PinholeImageMessage* message_image_right = dynamic_cast<srrg_core::PinholeImageMessage*>(sensor_message);
        camera_calibration_matrix                       = message_image_right->cameraMatrix();
        projection_matrix_right.block<3,3>(0, 0) = camera_calibration_matrix;
        projection_matrix_right.block<3,1>(0, 3) = camera_calibration_matrix*message_image_right->offset().inverse().translation();
      }
    }
    delete message;

    //ds check for termination
    if (projection_matrix_left.norm() > 0 && projection_matrix_right.norm() > 0) {
      break;
    }
  }
  message_reader.close();

  //ds info
  std::cerr << "camera calibration matrix: " << std::endl;
  std::cerr << camera_calibration_matrix << std::endl;
  std::cerr << "right projection matrix: " << std::endl;
  std::cerr << projection_matrix_right << std::endl;

  //ds currently active point tracks of the left camera (updated for every processed frame)
  std::vector<std::vector<const KeypointWithDescriptor*>> tracks;
  std::vector<KeypointWithDescriptor*> points_left_previous(0);
  cv::Mat image_left_previous;

  //ds left camera pose history per image
  std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> camera_left_to_world_per_image(0);

  //ds start processing
  message_reader.open(message_file);
  message = 0;
  while ((message = message_reader.readMessage())) {
    srrg_core::BaseSensorMessage* sensor_message = dynamic_cast<srrg_core::BaseSensorMessage*>(message);

    //ds add to synchronizer
    if (sensor_message->topic() == topic_camera_left) {
      synchronizer.putMessage(sensor_message);
    } else if (sensor_message->topic() == topic_camera_right) {
      synchronizer.putMessage(sensor_message);
    } else {
      delete sensor_message;
    }

    //ds if we have a synchronized package of sensor messages ready
    if (synchronizer.messagesReady()) {

      //ds buffer sensor data
      srrg_core::PinholeImageMessage* image_message_left  = dynamic_cast<srrg_core::PinholeImageMessage*>(synchronizer.messages()[0].get());
      srrg_core::PinholeImageMessage* image_message_right = dynamic_cast<srrg_core::PinholeImageMessage*>(synchronizer.messages()[1].get());
      const cv::Mat image_left(image_message_left->image());
      const cv::Mat image_right(image_message_right->image());

      //ds detect keypoints (right image is only used for validation)
      std::vector<cv::KeyPoint> keypoints_left;
      std::vector<cv::KeyPoint> keypoints_right;
      keypoint_detector->detect(image_left, keypoints_left);
      keypoint_detector->detect(image_right, keypoints_right);

      //ds compute descriptors (will automatically prune invalid keypoints)
      cv::Mat descriptors_left;
      cv::Mat descriptors_right;
      descriptor_extractor->compute(image_left, keypoints_left, descriptors_left);
      descriptor_extractor->compute(image_right, keypoints_right, descriptors_right);

      //ds assemble combined vectors (for stereo matching and easy tracking)
      std::vector<KeypointWithDescriptor*> points_left(keypoints_left.size());
      std::vector<KeypointWithDescriptor*> points_right(keypoints_right.size());
      for (uint32_t u = 0; u < keypoints_left.size(); ++u) {
        points_left[u] = new KeypointWithDescriptor(keypoints_left[u], descriptors_left.row(u));
      }
      for (uint32_t u = 0; u < keypoints_right.size(); ++u) {
        points_right[u] = new KeypointWithDescriptor(keypoints_right[u], descriptors_right.row(u));
      }
      std::cerr << "[" << image_message_left->seq() << "] features left: " << keypoints_left.size() << " right: " << keypoints_right.size();

      //ds get stereo matches (validation only)
      std::vector<std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>> stereo_matches(getMatches(points_left, points_right));
      std::cerr << " stereo matches: " << stereo_matches.size() << std::endl;

      //ds get current matches for tracks
      std::vector<std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>> track_matches(getMatches(points_left_previous, points_left));

      //ds info
      uint32_t number_of_added_tracks   = 0;
      uint32_t number_of_removed_tracks = 0;
      std::set<const KeypointWithDescriptor*> already_tracked_points;
      std::set<const KeypointWithDescriptor*> removed_tracked_points;

      //ds update track matrix
      std::vector<std::vector<const KeypointWithDescriptor*>> tracks_previous(tracks);
      tracks.clear();
      for (std::vector<const KeypointWithDescriptor*> track: tracks_previous) {

        //ds last point
        const KeypointWithDescriptor* point_last = track.back();
        const KeypointWithDescriptor* point_new  = 0;

        //ds check if we manage to track the point (last element in the track list)
        for (const std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>& match: track_matches) {

          //ds if we found a track - add the new point
          if (point_last == match.first) {
            point_new = match.second;
            break;
          }
        }

        //ds check for cancellation - free the whole track
        if (point_new == 0) {
          for (const KeypointWithDescriptor* point: track) {
            removed_tracked_points.insert(point);
            delete point;
          }
          ++number_of_removed_tracks;
          track.clear();
        } else {

          //ds add the new point
          track.push_back(point_new);
          already_tracked_points.insert(point_last);
          tracks.push_back(track);
        }
      }

      //ds add new tracks
      for (const std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>& match: track_matches) {

        //ds if not updated an existing track
        if (already_tracked_points.count(match.first) == 0) {

          //ds start a new track
          std::vector<const KeypointWithDescriptor*> track_new(2);
          track_new[0] = match.first;
          track_new[1] = match.second;
          tracks.push_back(track_new);
          ++number_of_added_tracks;
        }
      }
      std::cerr << "tracks: " << track_matches.size()
                << " removed: " << number_of_removed_tracks
                << " updated: " << already_tracked_points.size()
                << " added: " << number_of_added_tracks
                << " total: " << tracks.size() << std::endl;

      //ds draw current tracks
      if (tracks.size() > 0) {
        cv::Mat image_display_combined;
        cv::vconcat(image_left, image_left_previous, image_display_combined);
        cv::cvtColor(image_display_combined, image_display_combined, CV_GRAY2RGB);
        const cv::Point2f offset(0, image_left.rows);
        for (std::vector<const KeypointWithDescriptor*> track: tracks) {
          if (track.size() > 1) {

            //ds take previous and current point
            const KeypointWithDescriptor* point_last    = track[track.size()-2];
            const KeypointWithDescriptor* point_current = track[track.size()-1];

            //ds draw keypoints: from in red, to in blue
            cv::circle(image_display_combined, point_current->keypoint.pt, 3, cv::Scalar(255, 0, 0), 1);
            cv::circle(image_display_combined, point_last->keypoint.pt+offset, 3, cv::Scalar(0, 0, 255), 1);

            //ds draw connecting tracks
            cv::line(image_display_combined, point_current->keypoint.pt, point_last->keypoint.pt+offset, cv::Scalar(0, 255, 0), 1);

            //ds display track length on image as text
            cv::putText(image_display_combined,
                        std::to_string(point_current->identifier)+": "+std::to_string(track.size()),
                        point_current->keypoint.pt+cv::Point2f(5, 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 0, 255));
            cv::putText(image_display_combined,
                        std::to_string(point_last->identifier)+": "+std::to_string(track.size()),
                        point_last->keypoint.pt+offset+cv::Point2f(5, 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 0, 255));
          }
        }
        cv::imshow("tracks (top: new, bot: previous)", image_display_combined);
        cv::waitKey(0);
      }

      //ds grab ground truth pose
      const Eigen::Isometry3f camera_left_to_world(image_message_left->odometry());
      camera_left_to_world_per_image.push_back(camera_left_to_world);

      //ds stereo triangulate all available points (validation)
      stereo_points.clear();
      for (const std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>& match: stereo_matches) {
        if (std::fabs(match.second->keypoint.pt.x-match.first->keypoint.pt.x) > 0) {

          //ds compute depth (z in camera)
          const float depth_meters = projection_matrix_right(0,3)/(match.second->keypoint.pt.x-match.first->keypoint.pt.x);
          const float depth_meters_per_pixel = depth_meters/camera_calibration_matrix(0,0);

          //ds set 3d point
          const Eigen::Vector3f point_in_camera(depth_meters_per_pixel*(match.first->keypoint.pt.x-camera_calibration_matrix(0,2)),
                                                depth_meters_per_pixel*(match.first->keypoint.pt.y-camera_calibration_matrix(1,2)),
                                                depth_meters);
          stereo_points.insert(std::make_pair(match.first->identifier, camera_left_to_world*point_in_camera));
        }
      }
      std::cerr << "stereo points: " << stereo_points.size() << std::endl;

      //ds select tracks for monocular triangulation
      uint32_t number_of_triangulated_tracks = 0;
      for (const std::vector<const KeypointWithDescriptor*>& track: tracks) {
        if (track.size() > minimum_track_length_for_triangulation) {

          //ds grab related camera poses (aligned with track)
          std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> world_to_cameras_left(0);
          for (uint32_t u = camera_left_to_world_per_image.size()-track.size(); u < camera_left_to_world_per_image.size(); ++u) {
            world_to_cameras_left.push_back(camera_left_to_world_per_image[u].inverse());
          }

          //ds triangulate the point!
          const Eigen::Vector3f point_in_world = getPointTriangulatedLS(camera_calibration_matrix, track, world_to_cameras_left);
          ++number_of_triangulated_tracks;
        }
      }
      std::cerr << "triangulated tracks: " << number_of_triangulated_tracks << std::endl;

      //ds free untracked left points (from preceeding frame)
      for (const KeypointWithDescriptor* point: points_left_previous) {
        bool matched = false;
        for (const std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>& match: track_matches) {
          if (point == match.first) {
            matched = true;
            break;
          }
        }

        //ds if not matched and not already removed
        if (!matched && removed_tracked_points.count(point) == 0) {
          delete point;
        }
      }

      //ds update previous
      image_left_previous  = image_left;
      points_left_previous = points_left;

      //ds free all right points
      for (const KeypointWithDescriptor* point: points_right) {
        delete point;
      }
    }
  }
  return EXIT_SUCCESS;
}

std::vector<std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>> getMatches(const std::vector<KeypointWithDescriptor*>& points_a_,
                                                                                    const std::vector<KeypointWithDescriptor*>& points_b_) {

  //ds ids for current best distances (a previous point can match exactly one current point)
  std::map<uint32_t, std::pair<double, uint32_t>> best_matches;

  //ds find best matches for every current point (exhaustive)
  for (uint32_t b = 0; b < points_b_.size(); ++b) {
    uint32_t distance_best = maximum_matching_distance;
    uint32_t index_best_a  = 0;
    for (uint32_t a = 0; a < points_a_.size(); ++a) {
      const uint32_t distance = cv::norm(points_b_[b]->descriptor, points_a_[a]->descriptor, cv::NORM_HAMMING);
      if (distance < distance_best) {
        distance_best = distance;
        index_best_a  = a;
      }
    }
    if (distance_best < maximum_matching_distance) {

      //ds if already matched
      if (best_matches.find(index_best_a) != best_matches.end()) {

        //ds check if the current distance is lower
        if (distance_best < best_matches.at(index_best_a).first) {

          //ds update match
          best_matches.at(index_best_a).first  = distance_best;
          best_matches.at(index_best_a).second = b;
        }
      } else {
        best_matches.insert(std::make_pair(index_best_a, std::make_pair(distance_best, b)));
      }
    }
  }

  //ds compute actual matches from matrix
  std::vector<std::pair<KeypointWithDescriptor*, KeypointWithDescriptor*>> matches(0);
  for (const std::pair<uint32_t, std::pair<double, uint32_t>>& best_match: best_matches) {
    matches.push_back(std::make_pair(points_a_[best_match.first], points_b_[best_match.second.second]));
  }
  return matches;
}

Eigen::Vector3f getPointTriangulatedLS(const Eigen::Matrix3f& camera_calibration_matrix_,
                                       const std::vector<const KeypointWithDescriptor*>& track_,
                                       const std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>>& world_to_cameras_left_) {
  if (track_.size() != world_to_cameras_left_.size()) {
    throw std::runtime_error("invalid call");
  }

  //ds compute total distance of image coordinates
  float image_distance_traveled = 0;
  for (uint32_t u = 1; u < track_.size(); ++u) {
    image_distance_traveled += cv::norm(cv::Mat(track_[u-1]->keypoint.pt), cv::Mat(track_[u]->keypoint.pt));
  }

  //ds compute initial guess from first and last measurement
  const Eigen::Isometry3f& camera_previous_to_world = world_to_cameras_left_.front().inverse();
  const Eigen::Isometry3f& camera_current_to_world  = world_to_cameras_left_.back().inverse();
  const Eigen::Isometry3f camera_previous_to_current(camera_current_to_world.inverse()*camera_previous_to_world);
  std::pair<Eigen::Vector3f, Eigen::Vector3f> camera_points = getCameraPoints(track_.front()->keypoint.pt,
                                                                              track_.back()->keypoint.pt,
                                                                              camera_previous_to_current,
                                                                              camera_calibration_matrix_);

  //ds obtain world_point candidates
  const Eigen::Vector3f point_previous_in_world(camera_previous_to_world*camera_points.first);
  const Eigen::Vector3f point_current_in_world(camera_current_to_world*camera_points.second);

  //ds least squares setup - take initial guess from midpoint triangulation
  Eigen::Vector3f x((point_previous_in_world+point_current_in_world)/2);
  Eigen::Matrix3f H(Eigen::Matrix3f::Zero());
  Eigen::Vector3f b(Eigen::Vector3f::Zero());
  Eigen::Matrix2f omega(Eigen::Matrix2f::Identity());
  const uint32_t number_of_iterations                  = 100;
  const float minimum_error_difference_for_convergence = 1e-5;
  float total_error_squared_previous                   = 0;
  uint32_t number_of_inliers                           = 0;
  const float kernel_maximum_error_squared             = 25*25;

  //ds start iterations
  std::cerr << "----------------------------------------------------------------" << std::endl;
  std::cerr << "[i] mono x = [" << x.transpose() << "] measurements: " << track_.size() << std::endl;
  for (uint32_t i = 0; i < number_of_iterations; ++i) {
    H.setZero();
    b.setZero();
    float total_error_squared = 0;
    number_of_inliers         = 0;

    for (uint32_t index_measurement = 0; index_measurement < track_.size(); ++index_measurement) {
      const Eigen::Isometry3f& world_to_camera_left(world_to_cameras_left_[index_measurement]);
      const cv::Point2f& point_in_image_measured(track_[index_measurement]->keypoint.pt);

      //ds sample point into current image plane
      const Eigen::Vector3f point_in_camera_sampled = world_to_camera_left*x;
      const Eigen::Vector3f point_in_image_sampled  = camera_calibration_matrix_*point_in_camera_sampled;
      if (point_in_camera_sampled.z() <= 0) {
        continue;
      }

      //ds adjust information
      omega *= 1/point_in_camera_sampled.z();

      //ds obtain image coordinates
      const float u = point_in_image_sampled.x()/point_in_image_sampled.z();
      const float v = point_in_image_sampled.y()/point_in_image_sampled.z();

      //ds compute error
      const Eigen::Vector2f error(u-point_in_image_measured.x,
                                  v-point_in_image_measured.y);
      const float error_squared = error.transpose()*error;
      total_error_squared += error_squared;

      //ds robust kernel
      if (error_squared > kernel_maximum_error_squared) {
        omega *= kernel_maximum_error_squared/error_squared;
      } else {
        ++number_of_inliers;
      }

      //ds precompute constants
      const float inverse_sampled_c         = 1/point_in_image_sampled.z();
      const float inverse_sampled_c_squared = inverse_sampled_c*inverse_sampled_c;

      //ds compute jacobian
      Eigen::Matrix<float, 2, 3> jacobian_homogeneous_division;
      jacobian_homogeneous_division << inverse_sampled_c, 0, -point_in_image_sampled.x()*inverse_sampled_c_squared,
                                       0, inverse_sampled_c, -point_in_image_sampled.y()*inverse_sampled_c_squared;
      const Eigen::Matrix<float, 2, 3> jacobian = jacobian_homogeneous_division*camera_calibration_matrix_*world_to_camera_left.linear();
      const Eigen::Matrix<float, 3, 2> jacobian_transposed = jacobian.transpose();

      //ds update H and b
      H += jacobian_transposed*omega*jacobian;
      b += jacobian_transposed*omega*error;
    }

    //ds update solution with residual
    const Eigen::Vector3f dx = H.ldlt().solve(-b);
    x += dx;
    std::cerr << "[" << i << "] mono x = [" << x.transpose() << "] error: " << total_error_squared
              << " inliers: " << number_of_inliers << "/" << track_.size() << std::endl;

    //ds check for convergence
    if (std::fabs(total_error_squared-total_error_squared_previous) < minimum_error_difference_for_convergence) {
      break;
    } else {
      total_error_squared_previous = total_error_squared;
    }
  }

  if (number_of_inliers > track_.size()/3) {
    std::cerr << "point identifier: " << track_.back()->identifier
              << " track length: " << track_.size()
              << " total image distance: " << image_distance_traveled << std::endl;

    //ds check if we have a stereo point for comparison (validation only)
    if (stereo_points.find(track_.back()->identifier) != stereo_points.end()) {
      std::cerr << "stereo x = [" << stereo_points.at(track_.back()->identifier).transpose() << "]" << std::endl;
    } else {
      std::cerr << "stereo x not available" << std::endl;
    }
  } else {
    std::cerr << "not enough inliers, invalid solution - improve initial guess" << std::endl;
  }
  return x;
}

Eigen::Vector3f getImagePointNormalized(const cv::Point2f& point_, const Eigen::Matrix3f& camera_calibration_matrix_) {
  return Eigen::Vector3f((point_.x-camera_calibration_matrix_(0,2))/camera_calibration_matrix_(0,0),
                         (point_.y-camera_calibration_matrix_(1,2))/camera_calibration_matrix_(0,0),
                         1);
}

//ds retrieves camera points for two corresponding image points
std::pair<Eigen::Vector3f, Eigen::Vector3f> getCameraPoints(const cv::Point2f image_point_previous_,
                                                            const cv::Point2f image_point_current_,
                                                            const Eigen::Isometry3f& camera_previous_to_current_,
                                                            const Eigen::Matrix3f& camera_calibration_matrix_) {

  //ds readability
  const Eigen::Matrix<float, 1, 3>& r_1 = camera_previous_to_current_.linear().block<1,3>(0,0);
  const Eigen::Matrix<float, 1, 3>& r_2 = camera_previous_to_current_.linear().block<1,3>(1,0);
  const Eigen::Matrix<float, 1, 3>& r_3 = camera_previous_to_current_.linear().block<1,3>(2,0);

  //ds precomputations
  const float a_0 = (image_point_previous_.x-camera_calibration_matrix_(0,2))/camera_calibration_matrix_(0,0);
  const float b_0 = (image_point_previous_.y-camera_calibration_matrix_(1,2))/camera_calibration_matrix_(1,1);
  const float a_1 = (image_point_current_.x-camera_calibration_matrix_(0,2))/camera_calibration_matrix_(0,0);
  const float b_1 = (image_point_current_.y-camera_calibration_matrix_(1,2))/camera_calibration_matrix_(1,1);
  const Eigen::Vector3f x_0(a_0, b_0, 1);
  const Eigen::Vector3f x_1(a_1, b_1, 1);

  //ds build A matrix
  Eigen::Matrix<float, 3, 2> A(Eigen::Matrix<float, 3, 2>::Zero());
  A << -r_1*x_0, a_1,
       -r_2*x_0, b_1,
       -r_3*x_0, 1;

  //ds minimize squared error
  const Eigen::Vector2f z = A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(camera_previous_to_current_.translation());

  //ds return points
  return std::make_pair(x_0*z(0), x_1*z(0));
}
