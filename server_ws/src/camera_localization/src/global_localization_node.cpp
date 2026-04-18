#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

class CameraLocalizationNode : public rclcpp::Node
{
public:
  CameraLocalizationNode()
  : Node("global_localization_node")
  {
    // ----------------------------
    // Parameters
    // ----------------------------
    // Camera device: prefer a by-id path for stability, fall back to index.
    // Pass --ros-args -p camera_path:=/dev/video2  OR  -p camera_index:=2
    const auto camera_path  = declare_parameter<std::string>("camera_path", "");
    const auto camera_index = declare_parameter<int>("camera_index", 0);
    device_ = camera_path.empty() ? "/dev/video" + std::to_string(camera_index) : camera_path;

    width_ = declare_parameter<int>("width", 3840);
    height_ = declare_parameter<int>("height", 2160);
    fps_ = declare_parameter<int>("fps", 30);
    fourcc_ = declare_parameter<std::string>("fourcc", "MJPG");

    // V4L2 hardware controls (set after open to survive driver resets)
    camera_gain_ = declare_parameter<int>("camera_gain", -1);
    camera_gamma_ = declare_parameter<int>("camera_gamma", -1);

    // CLAHE preprocessing on every frame (before main ArUco detection)
    enable_clahe_preprocessing_ =
      declare_parameter<bool>("enable_clahe_preprocessing", false);
    clahe_clip_limit_ =
      declare_parameter<double>("clahe_clip_limit", 3.0);
    clahe_tile_size_ =
      declare_parameter<int>("clahe_tile_size", 16);

    // Calibration file: default resolves automatically from the installed pfe share dir.
    const auto calib_default = ament_index_cpp::get_package_share_directory("pfe")
      + "/camera_calibration/3840_2160_ELM12MP.yml";
    calibration_file_ = declare_parameter<std::string>("calibration_file", calib_default);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/camera/global_pose");
    detected_blocks_topic_ = declare_parameter<std::string>("detected_blocks_topic", "/detected_blocks");
    debug_view_ = declare_parameter<bool>("debug_view", true);

    // Marker IDs / sizes
    robot_marker_id_ = declare_parameter<int>("robot_marker_id", 1);
    table_marker_length_m_ = declare_parameter<double>("table_marker_length_m", 0.10);
    robot_marker_length_m_ = declare_parameter<double>("robot_marker_length_m", 0.07);
    block_marker_length_m_ = declare_parameter<double>("block_marker_length_m", 0.03);
    block_size_x_m_ = declare_parameter<double>("block_size_x_m", 0.15);
    block_size_y_m_ = declare_parameter<double>("block_size_y_m", 0.05);
    block_center_z_m_ = declare_parameter<double>("block_center_z_m", 0.03);

    const auto block_marker_ids =
      declare_parameter<std::vector<int64_t>>("block_marker_ids", std::vector<int64_t>{36, 47});
    const auto block_marker_colors =
      declare_parameter<std::vector<std::string>>("block_marker_colors", std::vector<std::string>{"bleu", "jaune"});

    if (block_marker_ids.size() != block_marker_colors.size()) {
      throw std::runtime_error(
              "block_marker_ids and block_marker_colors must have the same size");
    }

    block_ids_.clear();
    block_color_by_id_.clear();
    for (std::size_t i = 0; i < block_marker_ids.size(); ++i) {
      const int id = static_cast<int>(block_marker_ids[i]);
      block_ids_.insert(id);
      block_color_by_id_[id] = block_marker_colors[i];
    }

    // Fixed transform from robot marker frame to base_link frame.
    // Expressed in the MARKER frame.
    marker_to_base_x_ = declare_parameter<double>("marker_to_base_x", 0.0);
    marker_to_base_y_ = declare_parameter<double>("marker_to_base_y", 0.0);
    marker_to_base_z_ = declare_parameter<double>("marker_to_base_z", 0.0);
    marker_to_base_yaw_ = declare_parameter<double>("marker_to_base_yaw", 0.0);

    // Measurement covariance
    position_variance_x_ = declare_parameter<double>("position_variance_x", 0.01);
    position_variance_y_ = declare_parameter<double>("position_variance_y", 0.01);
    yaw_variance_ = declare_parameter<double>("yaw_variance", 0.03);

    min_table_markers_ = declare_parameter<int>("min_table_markers", 1);

    // ArUco detector tuning (configurable from YAML)
    detector_perspective_remove_pixel_per_cell_ =
      declare_parameter<int>("detector_perspective_remove_pixel_per_cell", 8);
    detector_adaptive_thresh_win_size_min_ =
      declare_parameter<int>("detector_adaptive_thresh_win_size_min", 3);
    detector_adaptive_thresh_win_size_max_ =
      declare_parameter<int>("detector_adaptive_thresh_win_size_max", 61);
    detector_adaptive_thresh_win_size_step_ =
      declare_parameter<int>("detector_adaptive_thresh_win_size_step", 10);
    detector_adaptive_thresh_constant_ =
      declare_parameter<double>("detector_adaptive_thresh_constant", 7.0);
    detector_min_marker_perimeter_rate_ =
      declare_parameter<double>("detector_min_marker_perimeter_rate", 0.005);
    detector_max_marker_perimeter_rate_ =
      declare_parameter<double>("detector_max_marker_perimeter_rate", 4.0);
    detector_error_correction_rate_ =
      declare_parameter<double>("detector_error_correction_rate", 0.35);
    detector_use_corner_refinement_subpix_ =
      declare_parameter<bool>("detector_use_corner_refinement_subpix", true);

    // Optional rescue pass focused on large table markers (IDs 20-23)
    enable_table_marker_rescue_pass_ =
      declare_parameter<bool>("enable_table_marker_rescue_pass", true);
    rescue_clahe_clip_limit_ =
      declare_parameter<double>("rescue_clahe_clip_limit", 2.0);
    rescue_adaptive_thresh_win_size_max_ =
      declare_parameter<int>("rescue_adaptive_thresh_win_size_max", 121);
    rescue_adaptive_thresh_win_size_step_ =
      declare_parameter<int>("rescue_adaptive_thresh_win_size_step", 20);
    rescue_min_marker_perimeter_rate_ =
      declare_parameter<double>("rescue_min_marker_perimeter_rate", 0.01);
    rescue_error_correction_rate_ =
      declare_parameter<double>("rescue_error_correction_rate", 0.5);

    // ----------------------------
    // Publisher
    // ----------------------------
    // Use RELIABLE QoS instead of default BEST_EFFORT to ensure delivery
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, qos);
    detected_blocks_pub_ = create_publisher<std_msgs::msg::String>(
      detected_blocks_topic_, qos);
    
    RCLCPP_INFO(get_logger(), "Publishers configured with RELIABLE QoS policy");

    // ----------------------------
    // Camera
    // ----------------------------
    openCamera();
    loadCalibration();
    initMarkerGeometry();
    initTableMarkerMap();
    initDetector();

    if (debug_view_) {
      cv::namedWindow("camera_localization_debug", cv::WINDOW_NORMAL);
    }

    const int period_ms = std::max(1, static_cast<int>(1000.0 / static_cast<double>(fps_)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CameraLocalizationNode::cameraTick, this));

    RCLCPP_INFO(get_logger(), "global_localization_node started");
    RCLCPP_INFO(get_logger(), "Publishing global pose on %s", pose_topic_.c_str());
  }

  ~CameraLocalizationNode() override
  {
    if (debug_view_) {
      cv::destroyAllWindows();
    }
  }

private:
  struct PoseGlobal
  {
    cv::Matx33d R_global_marker;
    cv::Vec3d t_global_marker;
  };

  struct DetectedEntity
  {
    int marker_id{0};
    std::string color;
    cv::Vec3d position_map{0.0, 0.0, 0.0};
    double yaw_rad{0.0};
    double size_x_m{0.0};
    double size_y_m{0.0};
    bool is_dynamic{true};
  };

  void openCamera()
  {
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      throw std::runtime_error("Failed to open camera: " + device_);
    }

    if (fourcc_.size() != 4) {
      throw std::runtime_error("fourcc must be exactly 4 characters");
    }

    cap_.set(cv::CAP_PROP_FOURCC,
             cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Apply V4L2 hardware controls if configured (gain, gamma)
    if (camera_gain_ >= 0) {
      cap_.set(cv::CAP_PROP_GAIN, camera_gain_);
      RCLCPP_INFO(get_logger(), "Set camera gain to %d (actual: %.0f)",
                  camera_gain_, cap_.get(cv::CAP_PROP_GAIN));
    }
    if (camera_gamma_ >= 0) {
      cap_.set(cv::CAP_PROP_GAMMA, camera_gamma_);
      RCLCPP_INFO(get_logger(), "Set camera gamma to %d (actual: %.0f)",
                  camera_gamma_, cap_.get(cv::CAP_PROP_GAMMA));
    }

    RCLCPP_INFO(get_logger(), "Opened camera %s", device_.c_str());

    RCLCPP_INFO(get_logger(), "Actual width: %.0f", cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    RCLCPP_INFO(get_logger(), "Actual height: %.0f", cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    RCLCPP_INFO(get_logger(), "Actual fps: %.2f", cap_.get(cv::CAP_PROP_FPS));

    int fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
    char fcc[] = {
      static_cast<char>(fourcc & 0xFF),
      static_cast<char>((fourcc >> 8) & 0xFF),
      static_cast<char>((fourcc >> 16) & 0xFF),
      static_cast<char>((fourcc >> 24) & 0xFF),
      '\0'
    };
    RCLCPP_INFO(get_logger(), "Actual FOURCC: %s", fcc);
  }

  void loadCalibration()
  {
    std::string calib_file = calibration_file_;
    if (!calib_file.empty() && calib_file[0] == '~') {
      const char* home = std::getenv("HOME");
      if (home) {
        calib_file = std::string(home) + calib_file.substr(1);
      }
    }
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      throw std::runtime_error("Could not open calibration file: " + calib_file);
    }

    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();

    if (camera_matrix_.empty()) {
      throw std::runtime_error("camera_matrix missing in calibration file");
    }

    if (dist_coeffs_.empty()) {
      throw std::runtime_error("distortion_coefficients missing in calibration file");
    }

    RCLCPP_INFO(get_logger(), "Loaded calibration from %s", calib_file.c_str());
  }

  void initMarkerGeometry()
  {
    obj_points_table_ = makeSquareObjectPoints(table_marker_length_m_);
    obj_points_robot_ = makeSquareObjectPoints(robot_marker_length_m_);
    obj_points_block_ = makeSquareObjectPoints(block_marker_length_m_);
  }

  cv::Mat makeSquareObjectPoints(double side_m) const
  {
    cv::Mat pts(4, 1, CV_32FC3);
    pts.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-side_m / 2.0f,  side_m / 2.0f, 0.0f);
    pts.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( side_m / 2.0f,  side_m / 2.0f, 0.0f);
    pts.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( side_m / 2.0f, -side_m / 2.0f, 0.0f);
    pts.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-side_m / 2.0f, -side_m / 2.0f, 0.0f);
    return pts;
  }

  void initTableMarkerMap()
  {
    const auto marker_ids =
      declare_parameter<std::vector<int64_t>>("table_marker_ids", std::vector<int64_t>{});
    const auto marker_x_m =
      declare_parameter<std::vector<double>>("table_marker_x_m", std::vector<double>{});
    const auto marker_y_m =
      declare_parameter<std::vector<double>>("table_marker_y_m", std::vector<double>{});
    const auto marker_z_m =
      declare_parameter<std::vector<double>>("table_marker_z_m", std::vector<double>{});
    const auto marker_yaw_rad =
      declare_parameter<std::vector<double>>("table_marker_yaw_rad", std::vector<double>{});

    table_ids_.clear();
    table_pose_global_.clear();

    if (marker_ids.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No table_marker_ids provided; using built-in default table marker map");

      // Assumes marker axes are aligned with map axes.
      const auto R_identity = cv::Matx33d::eye();
      table_ids_ = {20, 21, 22, 23};
      table_pose_global_[20] = PoseGlobal{R_identity, cv::Vec3d(0.6, 1.4, 0.0)};
      table_pose_global_[21] = PoseGlobal{R_identity, cv::Vec3d(2.4, 1.4, 0.0)};
      table_pose_global_[22] = PoseGlobal{R_identity, cv::Vec3d(0.6, 0.6, 0.0)};
      table_pose_global_[23] = PoseGlobal{R_identity, cv::Vec3d(2.4, 0.6, 0.0)};
      return;
    }

    const std::size_t count = marker_ids.size();
    if (marker_x_m.size() != count || marker_y_m.size() != count) {
      throw std::runtime_error(
              "table_marker_x_m and table_marker_y_m must have the same size as table_marker_ids");
    }

    if (!marker_z_m.empty() && marker_z_m.size() != count) {
      throw std::runtime_error(
              "table_marker_z_m must be empty or have the same size as table_marker_ids");
    }

    if (!marker_yaw_rad.empty() && marker_yaw_rad.size() != count) {
      throw std::runtime_error(
              "table_marker_yaw_rad must be empty or have the same size as table_marker_ids");
    }

    for (std::size_t i = 0; i < count; ++i) {
      const int id = static_cast<int>(marker_ids[i]);
      const double z = marker_z_m.empty() ? 0.0 : marker_z_m[i];
      const double yaw = marker_yaw_rad.empty() ? 0.0 : marker_yaw_rad[i];

      table_ids_.insert(id);
      table_pose_global_[id] = PoseGlobal{
        rotationZ(yaw),
        cv::Vec3d(marker_x_m[i], marker_y_m[i], z)
      };
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu table markers from parameters", table_ids_.size());
  }

  void initDetector()
  {
    detector_params_ = cv::aruco::DetectorParameters::create();

    detector_params_->perspectiveRemovePixelPerCell = detector_perspective_remove_pixel_per_cell_;
    detector_params_->adaptiveThreshWinSizeMin = detector_adaptive_thresh_win_size_min_;
    detector_params_->adaptiveThreshWinSizeMax = detector_adaptive_thresh_win_size_max_;
    detector_params_->adaptiveThreshWinSizeStep = detector_adaptive_thresh_win_size_step_;
    detector_params_->adaptiveThreshConstant = detector_adaptive_thresh_constant_;
    detector_params_->minMarkerPerimeterRate = detector_min_marker_perimeter_rate_;
    detector_params_->maxMarkerPerimeterRate = detector_max_marker_perimeter_rate_;
    detector_params_->errorCorrectionRate = static_cast<float>(detector_error_correction_rate_);
    detector_params_->cornerRefinementMethod = detector_use_corner_refinement_subpix_
      ? cv::aruco::CORNER_REFINE_SUBPIX
      : cv::aruco::CORNER_REFINE_NONE;

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  }

  void cameraTick()
  {
    auto tick_start = std::chrono::high_resolution_clock::now();
    frame_counter_++;
    
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      empty_frame_count_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty camera frame");
      return;
    }
    
    auto read_end = std::chrono::high_resolution_clock::now();
    double read_ms = std::chrono::duration<double, std::milli>(read_end - tick_start).count();
    last_frame_read_ms_ = read_ms;
    
    processFrame(frame);
    
    auto process_end = std::chrono::high_resolution_clock::now();
    double process_ms = std::chrono::duration<double, std::milli>(process_end - read_end).count();
    last_frame_process_ms_ = process_ms;
    double total_ms = std::chrono::duration<double, std::milli>(process_end - tick_start).count();
    
    if (frame_counter_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "[DIAGNOSTIC] Frames: %ld, Empty: %ld, Total: %.2f ms (read: %.2f ms, process: %.2f ms, json: %.2f ms, pub: %.2f ms)",
        frame_counter_, empty_frame_count_, total_ms, read_ms, process_ms,
        last_json_build_ms_, last_publish_ms_);
      RCLCPP_INFO(get_logger(),
        "[DETECTION] Initial: %d markers, Tables B4 rescue: %d, Tables after: %d, Rescue helped: %ld, solvePnP ok: %ld, fail: %ld",
        last_initial_marker_count_, last_table_markers_before_rescue_,
        last_table_markers_after_rescue_, rescue_pass_helped_,
        successful_solvepnp_, failed_solvepnp_);
      RCLCPP_INFO(get_logger(),
        "[MARKER 20] before rescue: %ld, after rescue: %ld, recovered by rescue: %ld",
        marker20_seen_before_rescue_,
        marker20_seen_after_rescue_,
        marker20_recovered_by_rescue_);
    }
  }

  void processFrame(const cv::Mat &frame)
  {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Apply CLAHE preprocessing for lighting-resilient detection
    if (enable_clahe_preprocessing_) {
      auto clahe = cv::createCLAHE(clahe_clip_limit_,
                                   cv::Size(clahe_tile_size_, clahe_tile_size_));
      clahe->apply(gray, gray);
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

    bool marker20_accepted_before_rescue = false;
    for (int id : ids) {
      if (id == 20) {
        marker20_accepted_before_rescue = true;
        break;
      }
    }

    int initial_marker_count = ids.size();
    int table_markers_before_rescue = 0;
    for (int id : ids) {
      if (table_ids_.count(id)) table_markers_before_rescue++;
    }

    runTableMarkerRescuePass(gray, ids, corners);

    bool marker20_accepted_after_rescue = false;
    for (int id : ids) {
      if (id == 20) {
        marker20_accepted_after_rescue = true;
        break;
      }
    }

    if (marker20_accepted_before_rescue) {
      marker20_seen_before_rescue_++;
    }
    if (marker20_accepted_after_rescue) {
      marker20_seen_after_rescue_++;
    }
    if (!marker20_accepted_before_rescue && marker20_accepted_after_rescue) {
      marker20_recovered_by_rescue_++;
    }

    int table_markers_after_rescue = 0;
    for (int id : ids) {
      if (table_ids_.count(id)) table_markers_after_rescue++;
    }
    
    last_initial_marker_count_ = initial_marker_count;
    last_table_markers_before_rescue_ = table_markers_before_rescue;
    last_table_markers_after_rescue_ = table_markers_after_rescue;
    if (table_markers_after_rescue > table_markers_before_rescue) {
      rescue_pass_helped_++;
    }

    cv::Mat debug_image;
  if (debug_view_) {
    debug_image = frame.clone();

  if (!ids.empty()) {
    cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
  }

}

if (ids.empty()) {
  publishDetectedEntities({});
  showDebug(debug_image);
  return;
}

    // -------------------------------------------------------
    // Step 1: Estimate camera pose in map using table markers
    // solvePnP gives: X_camera = R_camera_map * X_map + t_camera_map
    // -------------------------------------------------------
    std::vector<cv::Point3f> obj_pts_map;
    std::vector<cv::Point2f> img_pts;

    int table_marker_count = 0;

    for (size_t i = 0; i < ids.size(); ++i) {
      const int id = ids[i];
      if (!table_ids_.count(id)) {
        continue;
      }

      std::array<cv::Point3f, 4> corners_global;
      if (!getTableMarkerCornersGlobal(id, corners_global)) {
        continue;
      }

      for (int k = 0; k < 4; ++k) {
        obj_pts_map.push_back(corners_global[k]);
        img_pts.push_back(corners[i][k]);
      }
      table_marker_count++;
    }

    if (table_marker_count < min_table_markers_ || obj_pts_map.size() < 4) {
      failed_solvepnp_++;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Not enough table markers for camera pose (had %d, need %d)",
        table_marker_count, min_table_markers_);
      publishDetectedEntities({});
      showDebug(debug_image);
      return;
    }

    cv::Vec3d rvec_camera_map, tvec_camera_map;
    bool use_guess = have_camera_pose_guess_;

    if (use_guess) {
      rvec_camera_map = last_rvec_camera_map_;
      tvec_camera_map = last_tvec_camera_map_;
    }

    const bool ok_camera = cv::solvePnP(
      obj_pts_map,
      img_pts,
      camera_matrix_,
      dist_coeffs_,
      rvec_camera_map,
      tvec_camera_map,
      use_guess,
      cv::SOLVEPNP_ITERATIVE);

    if (!ok_camera) {
      have_camera_pose_guess_ = false;
      failed_solvepnp_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera solvePnP failed");
      publishDetectedEntities({});
      showDebug(debug_image);
      return;
    }

    successful_solvepnp_++;
    have_camera_pose_guess_ = true;
    last_rvec_camera_map_ = rvec_camera_map;
    last_tvec_camera_map_ = tvec_camera_map;

    cv::Mat Rcv_camera_map;
    cv::Rodrigues(rvec_camera_map, Rcv_camera_map);
    cv::Matx33d R_camera_map(Rcv_camera_map);

    // Invert to get map <- camera
    const cv::Matx33d R_map_camera = R_camera_map.t();
    const cv::Vec3d t_map_camera = -(R_map_camera * tvec_camera_map);

    if (debug_view_) {
      drawMapOrigin(debug_image, rvec_camera_map, tvec_camera_map);
      drawAxes(debug_image, rvec_camera_map, tvec_camera_map, 0.25f);
    }

    std::vector<DetectedEntity> detected_entities;
    detected_entities.reserve(ids.size());

    for (size_t i = 0; i < ids.size(); ++i) {
      const int id = ids[i];
      if (!block_ids_.count(id)) {
        continue;
      }

      cv::Matx33d R_map_block;
      cv::Vec3d t_map_block;
      if (!estimateMarkerPoseInMap(
            obj_points_block_, corners[i], R_map_camera, t_map_camera, R_map_block, t_map_block)) {
        continue;
      }

      DetectedEntity e;
      e.marker_id = id;
      e.color = block_color_by_id_[id];
      e.position_map = cv::Vec3d(t_map_block[0], t_map_block[1], block_center_z_m_);
      e.yaw_rad = std::atan2(R_map_block(1, 0), R_map_block(0, 0));
      e.size_x_m = block_size_x_m_;
      e.size_y_m = block_size_y_m_;
      e.is_dynamic = true;
      detected_entities.push_back(e);
    }

    // -------------------------------------------------------
    // Step 2: Find OUR robot marker
    // -------------------------------------------------------
    int robot_index = -1;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == robot_marker_id_) {
        robot_index = static_cast<int>(i);
        break;
      }
    }

    if (robot_index < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Robot marker ID %d not visible", robot_marker_id_);
      publishDetectedEntities(detected_entities);
      showDebug(debug_image);
      return;
    }

    // -------------------------------------------------------
    // Step 3: Estimate robot marker pose in camera frame
    // solvePnP gives: X_camera = R_camera_marker * X_marker + t_camera_marker
    // -------------------------------------------------------
    cv::Vec3d rvec_camera_marker, tvec_camera_marker;
    const bool ok_robot = cv::solvePnP(
      obj_points_robot_,
      corners[robot_index],
      camera_matrix_,
      dist_coeffs_,
      rvec_camera_marker,
      tvec_camera_marker,
      false,
      cv::SOLVEPNP_IPPE_SQUARE);

    if (!ok_robot) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot marker solvePnP failed");
      publishDetectedEntities(detected_entities);
      showDebug(debug_image);
      return;
    }

    cv::Mat Rcv_camera_marker;
    cv::Rodrigues(rvec_camera_marker, Rcv_camera_marker);
    cv::Matx33d R_camera_marker(Rcv_camera_marker);

    // -------------------------------------------------------
    // Step 4: Compose map <- marker
    // X_map = R_map_camera * (R_camera_marker * X_marker + t_camera_marker) + t_map_camera
    // So:
    // R_map_marker = R_map_camera * R_camera_marker
    // t_map_marker = R_map_camera * t_camera_marker + t_map_camera
    // -------------------------------------------------------
    const cv::Matx33d R_map_marker = R_map_camera * R_camera_marker;
    const cv::Vec3d t_map_marker = R_map_camera * tvec_camera_marker + t_map_camera;

    // -------------------------------------------------------
    // Step 5: Apply fixed marker -> base_link transform
    // T_map_base = T_map_marker * T_marker_base
    // -------------------------------------------------------
    const cv::Matx33d R_marker_base = rotationZ(marker_to_base_yaw_);
    const cv::Vec3d t_marker_base(marker_to_base_x_, marker_to_base_y_, marker_to_base_z_);

    const cv::Matx33d R_map_base = R_map_marker * R_marker_base;
    const cv::Vec3d t_map_base = R_map_marker * t_marker_base + t_map_marker;

    // Extract planar yaw in ROS ENU map frame
    const double yaw_map_base = std::atan2(R_map_base(1, 0), R_map_base(0, 0));

    DetectedEntity robot_entity;
    robot_entity.marker_id = robot_marker_id_;
    robot_entity.color = "robot";
    robot_entity.position_map = cv::Vec3d(t_map_base[0], t_map_base[1], t_map_base[2]);
    robot_entity.yaw_rad = yaw_map_base;
    robot_entity.size_x_m = 0.0;
    robot_entity.size_y_m = 0.0;
    robot_entity.is_dynamic = true;
    detected_entities.push_back(robot_entity);

    publishPose(t_map_base, yaw_map_base);
    publishDetectedEntities(detected_entities);

    if (debug_view_) {
      drawPoseText(debug_image, t_map_base, yaw_map_base);
      cv::drawFrameAxes(
        debug_image,
        camera_matrix_,
        dist_coeffs_,
        rvec_camera_marker,
        tvec_camera_marker,
        static_cast<float>(robot_marker_length_m_ * 1.5),
        2);
    }

    showDebug(debug_image);
  }

  void runTableMarkerRescuePass(
    const cv::Mat &gray,
    std::vector<int> &ids,
    std::vector<std::vector<cv::Point2f>> &corners)
  {
    if (!enable_table_marker_rescue_pass_) {
      return;
    }

    bool has_missing_table_markers = false;
    for (const int table_id : table_ids_) {
      if (std::find(ids.begin(), ids.end(), table_id) == ids.end()) {
        has_missing_table_markers = true;
        break;
      }
    }

    if (!has_missing_table_markers) {
      return;
    }

    cv::Mat gray_rescue;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(rescue_clahe_clip_limit_, cv::Size(16, 16));
    clahe->apply(gray, gray_rescue);

    cv::Ptr<cv::aruco::DetectorParameters> rescue_params = cv::aruco::DetectorParameters::create();
    *rescue_params = *detector_params_;
    rescue_params->adaptiveThreshWinSizeMax = rescue_adaptive_thresh_win_size_max_;
    rescue_params->adaptiveThreshWinSizeStep = rescue_adaptive_thresh_win_size_step_;
    rescue_params->minMarkerPerimeterRate = rescue_min_marker_perimeter_rate_;
    rescue_params->errorCorrectionRate = static_cast<float>(rescue_error_correction_rate_);

    std::vector<int> rescue_ids;
    std::vector<std::vector<cv::Point2f>> rescue_corners;
    std::vector<std::vector<cv::Point2f>> rescue_rejected;
    cv::aruco::detectMarkers(
      gray_rescue,
      dictionary_,
      rescue_corners,
      rescue_ids,
      rescue_params,
      rescue_rejected);

    int recovered_count = 0;
    for (std::size_t i = 0; i < rescue_ids.size(); ++i) {
      const int id = rescue_ids[i];
      if (!table_ids_.count(id)) {
        continue;
      }
      if (std::find(ids.begin(), ids.end(), id) != ids.end()) {
        continue;
      }

      ids.push_back(id);
      corners.push_back(rescue_corners[i]);
      recovered_count++;
    }

    if (recovered_count > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Recovered %d table marker(s) in rescue pass", recovered_count);
    }
  }

  bool estimateMarkerPoseInMap(
    const cv::Mat &obj_points,
    const std::vector<cv::Point2f> &img_corners,
    const cv::Matx33d &R_map_camera,
    const cv::Vec3d &t_map_camera,
    cv::Matx33d &R_map_marker,
    cv::Vec3d &t_map_marker) const
  {
    cv::Vec3d rvec_camera_marker;
    cv::Vec3d tvec_camera_marker;
    const bool ok = cv::solvePnP(
      obj_points,
      img_corners,
      camera_matrix_,
      dist_coeffs_,
      rvec_camera_marker,
      tvec_camera_marker,
      false,
      cv::SOLVEPNP_IPPE_SQUARE);

    if (!ok) {
      return false;
    }

    cv::Mat Rcv_camera_marker;
    cv::Rodrigues(rvec_camera_marker, Rcv_camera_marker);
    const cv::Matx33d R_camera_marker(Rcv_camera_marker);

    R_map_marker = R_map_camera * R_camera_marker;
    t_map_marker = R_map_camera * tvec_camera_marker + t_map_camera;
    return true;
  }

  static std::string jsonEscape(const std::string &value)
  {
    std::string escaped;
    escaped.reserve(value.size());
    for (char c : value) {
      if (c == '"' || c == '\\') {
        escaped.push_back('\\');
      }
      escaped.push_back(c);
    }
    return escaped;
  }

  void publishDetectedEntities(const std::vector<DetectedEntity> &entities)
  {
    auto pub_start = std::chrono::high_resolution_clock::now();
    
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << "[";

    for (std::size_t i = 0; i < entities.size(); ++i) {
      const auto &e = entities[i];
      if (i > 0) {
        ss << ",";
      }

      const double angle_z_deg = e.yaw_rad * 180.0 / M_PI;

      ss << "{";
      ss << "\"marker_id\":" << e.marker_id << ",";
      ss << "\"color\":\"" << jsonEscape(e.color) << "\",";
      ss << "\"x\":" << e.position_map[0] << ",";
      ss << "\"y\":" << e.position_map[1] << ",";
      ss << "\"z\":" << e.position_map[2] << ",";
      ss << "\"yaw\":" << e.yaw_rad << ",";
      ss << "\"angle_z_deg\":" << angle_z_deg << ",";
      ss << "\"size_x_m\":" << e.size_x_m << ",";
      ss << "\"size_y_m\":" << e.size_y_m << ",";
      ss << "\"dynamic\":" << (e.is_dynamic ? "true" : "false");
      ss << "}";
    }

    ss << "]";

    std_msgs::msg::String msg;
    msg.data = ss.str();
    
    auto pub_publish_start = std::chrono::high_resolution_clock::now();
    detected_blocks_pub_->publish(msg);
    auto pub_end = std::chrono::high_resolution_clock::now();
    
    double json_build_ms = std::chrono::duration<double, std::milli>(
      pub_publish_start - pub_start).count();
    double publish_ms = std::chrono::duration<double, std::milli>(
      pub_end - pub_publish_start).count();
    
    last_json_build_ms_ = json_build_ms;
    last_publish_ms_ = publish_ms;
  }

  cv::Matx33d rotationZ(double yaw) const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return cv::Matx33d(
      c, -s, 0.0,
      s,  c, 0.0,
      0.0, 0.0, 1.0);
  }

  bool getTableMarkerCornersGlobal(
    int id,
    std::array<cv::Point3f, 4> &corners_global) const
  {
    auto it = table_pose_global_.find(id);
    if (it == table_pose_global_.end()) {
      return false;
    }

    const PoseGlobal &pg = it->second;

    const std::array<cv::Point3f, 4> corners_marker = {
      cv::Point3f(-table_marker_length_m_ / 2.0f,  table_marker_length_m_ / 2.0f, 0.0f),
      cv::Point3f( table_marker_length_m_ / 2.0f,  table_marker_length_m_ / 2.0f, 0.0f),
      cv::Point3f( table_marker_length_m_ / 2.0f, -table_marker_length_m_ / 2.0f, 0.0f),
      cv::Point3f(-table_marker_length_m_ / 2.0f, -table_marker_length_m_ / 2.0f, 0.0f)
    };

    for (int k = 0; k < 4; ++k) {
      const cv::Vec3d p_marker(
        corners_marker[k].x,
        corners_marker[k].y,
        corners_marker[k].z);

      const cv::Vec3d p_global = pg.R_global_marker * p_marker + pg.t_global_marker;

      corners_global[k] = cv::Point3f(
        static_cast<float>(p_global[0]),
        static_cast<float>(p_global[1]),
        static_cast<float>(p_global[2]));
    }

    return true;
  }

  void publishPose(const cv::Vec3d &t_map_base, double yaw_map_base)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;

    msg.pose.pose.position.x = t_map_base[0];
    msg.pose.pose.position.y = t_map_base[1];
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_map_base);
    q.normalize();

    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // 6x6 covariance row-major:
    // [x y z roll pitch yaw]
    for (double &v : msg.pose.covariance) {
      v = 0.0;
    }

    msg.pose.covariance[0] = position_variance_x_;      // x
    msg.pose.covariance[7] = position_variance_y_;      // y
    msg.pose.covariance[14] = 1e3;                      // z unused
    msg.pose.covariance[21] = 1e3;                      // roll unused
    msg.pose.covariance[28] = 1e3;                      // pitch unused
    msg.pose.covariance[35] = yaw_variance_;            // yaw

    pose_pub_->publish(msg);
  }

  void drawAxes(
    cv::Mat &image,
    const cv::Vec3d &rvec_camera_map,
    const cv::Vec3d &tvec_camera_map,
    float axis_len)
  {
    std::vector<cv::Point3f> axes = {
      {0.f, 0.f, 0.f},
      {axis_len, 0.f, 0.f},
      {0.f, axis_len, 0.f},
      {0.f, 0.f, axis_len}
    };

    std::vector<cv::Point2f> proj;
    cv::projectPoints(
      axes,
      rvec_camera_map,
      tvec_camera_map,
      camera_matrix_,
      dist_coeffs_,
      proj);

    if (proj.size() == 4) {
      cv::arrowedLine(image, proj[0], proj[1], cv::Scalar(0, 0, 255), 3);   // X red
      cv::arrowedLine(image, proj[0], proj[2], cv::Scalar(0, 255, 0), 3);   // Y green
      cv::arrowedLine(image, proj[0], proj[3], cv::Scalar(255, 0, 0), 3);   // Z blue
      cv::circle(image, proj[0], 6, cv::Scalar(0, 255, 255), -1);
    }
  }

  void drawMapOrigin(
    cv::Mat &image,
    const cv::Vec3d &rvec_camera_map,
    const cv::Vec3d &tvec_camera_map) const
  {
    std::vector<cv::Point3f> map_origin = {{0.f, 0.f, 0.f}};
    std::vector<cv::Point2f> proj;
    cv::projectPoints(
      map_origin,
      rvec_camera_map,
      tvec_camera_map,
      camera_matrix_,
      dist_coeffs_,
      proj);

    if (proj.size() != 1) {
      return;
    }

    const cv::Point origin_pt = proj[0];
    cv::circle(image, origin_pt, 9, cv::Scalar(0, 255, 255), -1);
    cv::circle(image, origin_pt, 14, cv::Scalar(0, 0, 0), 2);
    cv::putText(
      image,
      "map origin (0,0)",
      origin_pt + cv::Point(14, -12),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(0, 255, 255),
      2);
  }

  void drawPoseText(cv::Mat &image, const cv::Vec3d &t_map_base, double yaw) const
  {
    const std::string line1 =
      "base_link map x=" + std::to_string(t_map_base[0]).substr(0, 5) +
      " y=" + std::to_string(t_map_base[1]).substr(0, 5);
    const std::string line2 =
      "yaw=" + std::to_string(yaw).substr(0, 6);

    cv::putText(image, line1, cv::Point(30, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
    cv::putText(image, line2, cv::Point(30, 80),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
  }

  void showDebug(const cv::Mat &image)
  {
    if (!debug_view_ || image.empty()) {
      return;
    }

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(1280, 720));
    cv::imshow("camera_localization_debug", resized);

    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      rclcpp::shutdown();
    }
  }

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detected_blocks_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Camera / calibration
  cv::VideoCapture cap_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // ArUco
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  // Marker geometry
  cv::Mat obj_points_table_;
  cv::Mat obj_points_robot_;
  cv::Mat obj_points_block_;

  // Known map markers
  std::unordered_set<int> table_ids_;
  std::unordered_set<int> block_ids_;
  std::unordered_map<int, std::string> block_color_by_id_;
  std::unordered_map<int, PoseGlobal> table_pose_global_;

  // Camera pose guess for faster/stabler solvePnP
  bool have_camera_pose_guess_{false};
  cv::Vec3d last_rvec_camera_map_{0.0, 0.0, 0.0};
  cv::Vec3d last_tvec_camera_map_{0.0, 0.0, 0.0};
  
  // Diagnostics: frame counting and timing
  uint64_t frame_counter_{0};
  uint64_t empty_frame_count_{0};
  double last_frame_read_ms_{0.0};
  double last_frame_process_ms_{0.0};
  double last_json_build_ms_{0.0};
  double last_publish_ms_{0.0};
  
  // Detection diagnostics
  int last_initial_marker_count_{0};
  int last_table_markers_before_rescue_{0};
  int last_table_markers_after_rescue_{0};
  uint64_t rescue_pass_helped_{0};
  uint64_t successful_solvepnp_{0};
  uint64_t failed_solvepnp_{0};

  uint64_t marker20_seen_before_rescue_{0};
  uint64_t marker20_seen_after_rescue_{0};
  uint64_t marker20_recovered_by_rescue_{0};

  // Parameters
  std::string device_;
  std::string fourcc_;
  std::string calibration_file_;
  std::string map_frame_;
  std::string pose_topic_;
  std::string detected_blocks_topic_;
  int width_{0};
  int height_{0};
  int fps_{30};
  int camera_gain_{-1};
  int camera_gamma_{-1};
  bool enable_clahe_preprocessing_{false};
  double clahe_clip_limit_{3.0};
  int clahe_tile_size_{16};
  bool debug_view_{true};

  int robot_marker_id_{1};
  int min_table_markers_{1};

  double table_marker_length_m_{0.10};
  double robot_marker_length_m_{0.07};
  double block_marker_length_m_{0.03};
  double block_size_x_m_{0.15};
  double block_size_y_m_{0.05};
  double block_center_z_m_{0.03};

  double marker_to_base_x_{0.0};
  double marker_to_base_y_{0.0};
  double marker_to_base_z_{0.0};
  double marker_to_base_yaw_{0.0};

  double position_variance_x_{0.01};
  double position_variance_y_{0.01};
  double yaw_variance_{0.03};

  int detector_perspective_remove_pixel_per_cell_{8};
  int detector_adaptive_thresh_win_size_min_{3};
  int detector_adaptive_thresh_win_size_max_{61};
  int detector_adaptive_thresh_win_size_step_{10};
  double detector_adaptive_thresh_constant_{7.0};
  double detector_min_marker_perimeter_rate_{0.005};
  double detector_max_marker_perimeter_rate_{4.0};
  double detector_error_correction_rate_{0.35};
  bool detector_use_corner_refinement_subpix_{true};

  bool enable_table_marker_rescue_pass_{true};
  double rescue_clahe_clip_limit_{2.0};
  int rescue_adaptive_thresh_win_size_max_{121};
  int rescue_adaptive_thresh_win_size_step_{20};
  double rescue_min_marker_perimeter_rate_{0.01};
  double rescue_error_correction_rate_{0.5};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
