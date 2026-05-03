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
#include <atomic>
#include <mutex>
#include <thread>
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
    // Pass --ros-args -p camera_path:=/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0
    // OR -p camera_index:=4
    const auto camera_path  = declare_parameter<std::string>(
      "camera_path", "/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera_01.00.00-video-index0");
    const auto camera_index = declare_parameter<int>("camera_index", 0);
    device_ = camera_path.empty() ? "/dev/video" + std::to_string(camera_index) : camera_path;

    width_ = declare_parameter<int>("width", 3840);
    height_ = declare_parameter<int>("height", 2160);
    fps_ = declare_parameter<int>("fps", 30);
    fourcc_ = declare_parameter<std::string>("fourcc", "MJPG");

    // V4L2 hardware controls (set after open to survive driver resets)
    camera_gain_ = declare_parameter<int>("camera_gain", -1);
    camera_gamma_ = declare_parameter<int>("camera_gamma", -1);
    camera_backlight_compensation_ =
      declare_parameter<int>("camera_backlight_compensation", -1);
    camera_autofocus_ = declare_parameter<int>("camera_autofocus", -1);
    camera_focus_absolute_ = declare_parameter<int>("camera_focus_absolute", -1);
    camera_exposure_time_absolute_ =
      declare_parameter<int>("camera_exposure_time_absolute", -1);

    // CLAHE preprocessing on every frame (before main ArUco detection)
    enable_clahe_preprocessing_ =
      declare_parameter<bool>("enable_clahe_preprocessing", false);
    clahe_clip_limit_ =
      declare_parameter<double>("clahe_clip_limit", 3.0);
    clahe_tile_size_ =
      declare_parameter<int>("clahe_tile_size", 16);

    // Calibration file: default resolves automatically from the installed camera_calibration share dir.
    const auto calib_default = ament_index_cpp::get_package_share_directory("camera_calibration")
      + "/calibration_files/calibration_mec.yml";
    calibration_file_ = declare_parameter<std::string>("calibration_file", calib_default);

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/camera/global_pose");
    detected_blocks_topic_ = declare_parameter<std::string>("detected_blocks_topic", "/detected_blocks");
    debug_view_ = declare_parameter<bool>("debug_view", true);

    // Marker IDs / sizes
    robot_marker_id_ = declare_parameter<int>("robot_marker_id", 1);
    enemy_robot_marker_id_ = declare_parameter<int>("enemy_robot_marker_id", 6);
    table_marker_length_m_ = declare_parameter<double>("table_marker_length_m", 0.10);
    robot_marker_length_m_ = declare_parameter<double>("robot_marker_length_m", 0.07);
    enemy_robot_marker_length_m_ = declare_parameter<double>("enemy_robot_marker_length_m", 0.07);
    enemy_robot_size_x_m_ = declare_parameter<double>("enemy_robot_size_x_m", 0.35);
    enemy_robot_size_y_m_ = declare_parameter<double>("enemy_robot_size_y_m", 0.35);
    enemy_robot_obstacle_z_m_ = declare_parameter<double>("enemy_robot_obstacle_z_m", 0.05);
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

    // Fixed transform from base_link to the robot ArUco marker.
    // These are the values you measure on the robot: marker pose expressed in base_link.
    base_link_to_aruco_x_ = declare_parameter<double>("base_link_to_aruco_x", 0.0);
    base_link_to_aruco_y_ = declare_parameter<double>("base_link_to_aruco_y", 0.0);
    base_link_to_aruco_z_ = declare_parameter<double>("base_link_to_aruco_z", 0.0);
    base_link_to_aruco_yaw_ = declare_parameter<double>("base_link_to_aruco_yaw", 0.0);

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

    // Optional rescue pass for block markers near frame borders (IDs 36/47)
    enable_block_marker_border_rescue_pass_ =
      declare_parameter<bool>("enable_block_marker_border_rescue_pass", true);
    block_rescue_pad_ratio_ =
      declare_parameter<double>("block_rescue_pad_ratio", 0.06);
    block_rescue_clahe_clip_limit_ =
      declare_parameter<double>("block_rescue_clahe_clip_limit", 3.0);
    block_rescue_adaptive_thresh_win_size_max_ =
      declare_parameter<int>("block_rescue_adaptive_thresh_win_size_max", 181);
    block_rescue_adaptive_thresh_win_size_step_ =
      declare_parameter<int>("block_rescue_adaptive_thresh_win_size_step", 30);
    block_rescue_min_marker_perimeter_rate_ =
      declare_parameter<double>("block_rescue_min_marker_perimeter_rate", 0.003);
    block_rescue_error_correction_rate_ =
      declare_parameter<double>("block_rescue_error_correction_rate", 0.6);
    block_rescue_min_distance_to_border_ =
      declare_parameter<int>("block_rescue_min_distance_to_border", 0);

    // Bird's-eye-view rescue pass for block markers (requires valid pose)
    enable_bev_block_rescue_pass_ =
      declare_parameter<bool>("enable_bev_block_rescue_pass", true);
    bev_block_rescue_scale_px_per_m_ =
      declare_parameter<double>("bev_block_rescue_scale_px_per_m", 1000.0);

    // Snapshot-based pose stabilization
    pose_update_interval_s_ =
      declare_parameter<double>("pose_update_interval_s", 5.0);

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
      display_timer_ = create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&CameraLocalizationNode::displayTick, this));
    }

    running_.store(true, std::memory_order_release);
    processing_thread_ = std::thread(&CameraLocalizationNode::processingLoop, this);

    RCLCPP_INFO(get_logger(), "global_localization_node started");
    RCLCPP_INFO(get_logger(), "Publishing global pose on %s", pose_topic_.c_str());
  }

  ~CameraLocalizationNode() override
  {
    running_.store(false, std::memory_order_release);
    cap_.release();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
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
    if (camera_backlight_compensation_ >= 0) {
      cap_.set(cv::CAP_PROP_BACKLIGHT, camera_backlight_compensation_);
      RCLCPP_INFO(get_logger(), "Set camera backlight compensation to %d (actual: %.0f)",
                  camera_backlight_compensation_, cap_.get(cv::CAP_PROP_BACKLIGHT));
    }
    if (camera_autofocus_ >= 0) {
      cap_.set(cv::CAP_PROP_AUTOFOCUS, camera_autofocus_);
      RCLCPP_INFO(get_logger(), "Set camera autofocus to %d (actual: %.0f)",
                  camera_autofocus_, cap_.get(cv::CAP_PROP_AUTOFOCUS));
    }
    if (camera_focus_absolute_ >= 0) {
      cap_.set(cv::CAP_PROP_FOCUS, camera_focus_absolute_);
      RCLCPP_INFO(get_logger(), "Set camera focus to %d (actual: %.0f)",
                  camera_focus_absolute_, cap_.get(cv::CAP_PROP_FOCUS));
    }
    if (camera_exposure_time_absolute_ >= 0) {
      cap_.set(cv::CAP_PROP_EXPOSURE, camera_exposure_time_absolute_);
      RCLCPP_INFO(get_logger(), "Set camera exposure time absolute to %d (actual: %.0f)",
                  camera_exposure_time_absolute_, cap_.get(cv::CAP_PROP_EXPOSURE));
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
    obj_points_enemy_robot_ = makeSquareObjectPoints(enemy_robot_marker_length_m_);
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

  void processingLoop()
  {
    while (running_.load(std::memory_order_acquire) && rclcpp::ok()) {
      cameraTick();
    }
  }

  void cameraTick()
  {
    auto tick_start = std::chrono::high_resolution_clock::now();
    frame_counter_++;
    
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      empty_frame_count_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty camera frame");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
    runBlockMarkerBorderRescuePass(gray, ids, corners);

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
    // Snapshot-based pose: only re-solve every pose_update_interval_s_ seconds.
    // Prefer the pose from when all 4 table markers were visible ("golden").
    // Keep the very first successful pose as a fallback.
    // -------------------------------------------------------
    auto now = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(now - last_pose_update_time_).count();
    bool should_update_pose = !have_active_pose_ || elapsed_s >= pose_update_interval_s_;

    if (should_update_pose) {
      std::vector<cv::Point3f> obj_pts_map;
      std::vector<cv::Point2f> img_pts;
      int table_marker_count = 0;

      for (size_t i = 0; i < ids.size(); ++i) {
        const int id = ids[i];
        if (!table_ids_.count(id)) continue;

        std::array<cv::Point3f, 4> corners_global;
        if (!getTableMarkerCornersGlobal(id, corners_global)) continue;

        for (int k = 0; k < 4; ++k) {
          obj_pts_map.push_back(corners_global[k]);
          img_pts.push_back(corners[i][k]);
        }
        table_marker_count++;
      }

      if (table_marker_count >= min_table_markers_ && obj_pts_map.size() >= 4) {
        cv::Vec3d rvec_camera_map, tvec_camera_map;
        bool use_guess = have_camera_pose_guess_;
        if (use_guess) {
          rvec_camera_map = last_rvec_camera_map_;
          tvec_camera_map = last_tvec_camera_map_;
        }

        const bool ok_camera = cv::solvePnP(
          obj_pts_map, img_pts, camera_matrix_, dist_coeffs_,
          rvec_camera_map, tvec_camera_map, use_guess, cv::SOLVEPNP_ITERATIVE);

        if (ok_camera) {
          successful_solvepnp_++;
          have_camera_pose_guess_ = true;
          last_rvec_camera_map_ = rvec_camera_map;
          last_tvec_camera_map_ = tvec_camera_map;
          last_pose_update_time_ = now;

          // First-ever successful solve → save as fallback
          if (!have_fallback_pose_) {
            fallback_rvec_ = rvec_camera_map;
            fallback_tvec_ = tvec_camera_map;
            have_fallback_pose_ = true;
            active_rvec_ = rvec_camera_map;
            active_tvec_ = tvec_camera_map;
            have_active_pose_ = true;
            RCLCPP_INFO(get_logger(), "Saved initial fallback pose (%d table markers)", table_marker_count);
          }

          // All 4 table markers visible → update golden pose
          if (table_marker_count == 4) {
            golden_rvec_ = rvec_camera_map;
            golden_tvec_ = tvec_camera_map;
            have_golden_pose_ = true;
            active_rvec_ = rvec_camera_map;
            active_tvec_ = tvec_camera_map;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
              "Updated golden pose (all 4 table markers)");
          } else if (!have_golden_pose_) {
            // No golden pose yet, use whatever we have
            active_rvec_ = rvec_camera_map;
            active_tvec_ = tvec_camera_map;
          }
          // If we have golden but <4 markers → keep using golden (don't update)
        } else {
          have_camera_pose_guess_ = false;
          failed_solvepnp_++;
        }
      }
    }

    // If we still have no usable pose, bail out
    if (!have_active_pose_) {
      failed_solvepnp_++;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No camera pose available yet (waiting for table markers)");
      publishDetectedEntities({});
      showDebug(debug_image);
      return;
    }

    // Update BEV homography from current active pose, then run BEV block rescue.
    updateBevHomography();
    runBirdEyeBlockRescuePass(gray, ids, corners);

    // Use the active (golden or fallback) pose
    cv::Mat Rcv_camera_map;
    cv::Rodrigues(active_rvec_, Rcv_camera_map);
    cv::Matx33d R_camera_map(Rcv_camera_map);

    // Invert to get map <- camera
    const cv::Matx33d R_map_camera = R_camera_map.t();
    const cv::Vec3d t_map_camera = -(R_map_camera * active_tvec_);

    if (debug_view_) {
      drawMapOrigin(debug_image, active_rvec_, active_tvec_);
      drawAxes(debug_image, active_rvec_, active_tvec_, 0.25f);
    }

    std::vector<DetectedEntity> detected_entities;
    detected_entities.reserve(ids.size());

    for (size_t i = 0; i < ids.size(); ++i) {
      const int id = ids[i];
      if (!block_ids_.count(id)) {
        continue;
      }

      const cv::Point2f center_px = 0.25f * (
        corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]);

      cv::Point2d xy_map;
      if (!pixelToMapXY(center_px, R_map_camera, t_map_camera, block_center_z_m_, xy_map)) {
        continue;
      }

      double yaw_map = 0.0;
      if (!yawFromMarkerCornersMap(
            corners[i], R_map_camera, t_map_camera, block_center_z_m_, yaw_map)) {
        continue;
      }

      DetectedEntity e;
      e.marker_id = id;
      e.color = block_color_by_id_[id];
      e.position_map = cv::Vec3d(xy_map.x, xy_map.y, block_center_z_m_);
      e.yaw_rad = yaw_map;
      e.size_x_m = block_size_x_m_;
      e.size_y_m = block_size_y_m_;
      e.is_dynamic = true;
      detected_entities.push_back(e);
    }

    for (size_t i = 0; i < ids.size(); ++i) {
      const int id = ids[i];
      if (id != enemy_robot_marker_id_) {
        continue;
      }

      cv::Matx33d R_map_enemy;
      cv::Vec3d t_map_enemy;
      if (!estimateMarkerPoseInMap(
            obj_points_enemy_robot_,
            corners[i],
            R_map_camera,
            t_map_camera,
            R_map_enemy,
            t_map_enemy)) {
        continue;
      }

      DetectedEntity enemy_entity;
      enemy_entity.marker_id = id;
      enemy_entity.color = "enemy_robot";
      enemy_entity.position_map = cv::Vec3d(
        t_map_enemy[0],
        t_map_enemy[1],
        enemy_robot_obstacle_z_m_);
      enemy_entity.yaw_rad = std::atan2(R_map_enemy(1, 0), R_map_enemy(0, 0));
      enemy_entity.size_x_m = enemy_robot_size_x_m_;
      enemy_entity.size_y_m = enemy_robot_size_y_m_;
      enemy_entity.is_dynamic = true;
      detected_entities.push_back(enemy_entity);
      break;
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

    // Fix Z-axis ambiguity: marker Z should point toward camera (positive Z in camera frame).
    // If it points away (R's 3rd column Z < 0), flip by rotating 180° around X.
    if (Rcv_camera_marker.at<double>(2, 2) < 0) {
      // Flip Z and Y axes (180° rotation around X)
      Rcv_camera_marker.col(1) *= -1.0;
      Rcv_camera_marker.col(2) *= -1.0;
      cv::Rodrigues(Rcv_camera_marker, rvec_camera_marker);
    }

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
    // Step 5: Convert detected marker pose into base_link pose.
    // The measured robot mounting transform is T_base_aruco, so:
    // T_map_base = T_map_aruco * inverse(T_base_aruco)
    // -------------------------------------------------------
    const cv::Matx33d R_base_aruco = rotationZ(base_link_to_aruco_yaw_);
    const cv::Vec3d t_base_aruco(
      base_link_to_aruco_x_,
      base_link_to_aruco_y_,
      base_link_to_aruco_z_);

    const cv::Matx33d R_aruco_base = R_base_aruco.t();
    const cv::Vec3d t_aruco_base = -(R_aruco_base * t_base_aruco);

    const cv::Matx33d R_map_base = R_map_marker * R_aruco_base;
    const cv::Vec3d t_map_base = R_map_marker * t_aruco_base + t_map_marker;

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

  // Compute and cache the homography from image pixels to a flat bird's-eye-view
  // image of the table (z = block_center_z_m plane).
  // H maps image pixel (u,v,1) to BEV pixel (x,y,w).
  // Uses the pinhole model without distortion (good approximation for small distortion).
  void updateBevHomography()
  {
    if (!have_active_pose_) { bev_H_valid_ = false; return; }

    cv::Mat R_cv;
    cv::Rodrigues(active_rvec_, R_cv);

    const double h = block_center_z_m_;
    const double s = bev_block_rescue_scale_px_per_m_;

    // H_map_to_img: maps table (X,Y) at z=h to image pixel (u,v,w).
    // Point [X,Y,h]^T in map -> camera: R*[X,Y,h] + t = X*r1 + Y*r2 + (h*r3 + t)
    cv::Mat r1 = R_cv.col(0).clone();
    cv::Mat r2 = R_cv.col(1).clone();
    cv::Mat r3 = R_cv.col(2).clone();
    cv::Mat tvec = cv::Mat(active_tvec_);
    cv::Mat col3 = h * r3 + tvec;

    cv::Mat H34(3, 3, CV_64F);
    r1.copyTo(H34.col(0));
    r2.copyTo(H34.col(1));
    col3.copyTo(H34.col(2));

    cv::Mat H_map_to_img = cv::Mat(camera_matrix_) * H34;  // 3x3

    // Scale from map (meters) to BEV pixels
    cv::Mat S = (cv::Mat_<double>(3, 3) <<
      s, 0, 0,
      0, s, 0,
      0, 0, 1);

    // H_img_to_bev = S * H_map_to_img^{-1}
    bev_H_     = S * H_map_to_img.inv();
    bev_H_inv_ = bev_H_.inv();
    bev_H_valid_ = true;
  }

  void runBirdEyeBlockRescuePass(
    const cv::Mat &gray,
    std::vector<int> &ids,
    std::vector<std::vector<cv::Point2f>> &corners)
  {
    if (!enable_bev_block_rescue_pass_ || !bev_H_valid_) { return; }

    const int bev_w = static_cast<int>(std::round(3.0 * bev_block_rescue_scale_px_per_m_));
    const int bev_h = static_cast<int>(std::round(2.0 * bev_block_rescue_scale_px_per_m_));

    cv::Mat bev_gray;
    cv::warpPerspective(gray, bev_gray, bev_H_, cv::Size(bev_w, bev_h));

    cv::Mat bev_enhanced;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(block_rescue_clahe_clip_limit_, cv::Size(16, 16));
    clahe->apply(bev_gray, bev_enhanced);

    // Scale min perimeter rate to BEV image size
    const double bev_min_perim_rate =
      (block_marker_length_m_ * 4.0 * bev_block_rescue_scale_px_per_m_) /
      (4.0 * static_cast<double>(std::min(bev_w, bev_h))) * 0.5;

    cv::Ptr<cv::aruco::DetectorParameters> bev_params = cv::aruco::DetectorParameters::create();
    *bev_params = *detector_params_;
    bev_params->adaptiveThreshWinSizeMax = block_rescue_adaptive_thresh_win_size_max_;
    bev_params->adaptiveThreshWinSizeStep = block_rescue_adaptive_thresh_win_size_step_;
    bev_params->minMarkerPerimeterRate = bev_min_perim_rate;
    bev_params->errorCorrectionRate = static_cast<float>(block_rescue_error_correction_rate_);
    bev_params->minDistanceToBorder = 0;

    std::vector<int> bev_ids;
    std::vector<std::vector<cv::Point2f>> bev_corners;
    cv::aruco::detectMarkers(bev_enhanced, dictionary_, bev_corners, bev_ids, bev_params);

    constexpr float kMinInstanceDistSq = 80.0f * 80.0f;
    int recovered_count = 0;

    for (std::size_t i = 0; i < bev_ids.size(); ++i) {
      const int id = bev_ids[i];
      if (!block_ids_.count(id)) { continue; }

      // Map corners back to image space
      std::vector<cv::Point2f> img_corners(4);
      cv::perspectiveTransform(bev_corners[i], img_corners, bev_H_inv_);

      // Compute centre in image space for dedup
      cv::Point2f centre(0.f, 0.f);
      for (auto &pt : img_corners) centre += pt;
      centre *= 0.25f;

      // Clamp to image bounds
      for (auto &pt : img_corners) {
        pt.x = std::clamp(pt.x, 0.0f, static_cast<float>(gray.cols - 1));
        pt.y = std::clamp(pt.y, 0.0f, static_cast<float>(gray.rows - 1));
      }

      // Skip duplicates (same ID already detected nearby)
      bool duplicate = false;
      for (std::size_t j = 0; j < ids.size(); ++j) {
        if (ids[j] != id) { continue; }
        cv::Point2f ec(0.f, 0.f);
        for (auto &pt : corners[j]) ec += pt;
        ec *= 0.25f;
        float dx = centre.x - ec.x, dy = centre.y - ec.y;
        if (dx * dx + dy * dy < kMinInstanceDistSq) { duplicate = true; break; }
      }
      if (duplicate) { continue; }

      ids.push_back(id);
      corners.push_back(img_corners);
      recovered_count++;
    }

    if (recovered_count > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "BEV rescue recovered %d block marker(s)", recovered_count);
    }
  }

  void runBlockMarkerBorderRescuePass(
    const cv::Mat &gray,
    std::vector<int> &ids,
    std::vector<std::vector<cv::Point2f>> &corners)
  {
    if (!enable_block_marker_border_rescue_pass_) {
      return;
    }

    // Always run the rescue pass for block markers — it adds instances of IDs that
    // are only partially visible at the frame border, even if another instance of
    // the same ID was already found in the main pass.

    const int min_dim = std::min(gray.cols, gray.rows);
    const int pad_px = std::max(8, static_cast<int>(std::round(block_rescue_pad_ratio_ * min_dim)));

    cv::Mat padded;
    cv::copyMakeBorder(
      gray, padded,
      pad_px, pad_px, pad_px, pad_px,
      cv::BORDER_REPLICATE);

    cv::Mat padded_rescue;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(block_rescue_clahe_clip_limit_, cv::Size(16, 16));
    clahe->apply(padded, padded_rescue);

    cv::Ptr<cv::aruco::DetectorParameters> rescue_params = cv::aruco::DetectorParameters::create();
    *rescue_params = *detector_params_;
    rescue_params->adaptiveThreshWinSizeMax = block_rescue_adaptive_thresh_win_size_max_;
    rescue_params->adaptiveThreshWinSizeStep = block_rescue_adaptive_thresh_win_size_step_;
    rescue_params->minMarkerPerimeterRate = block_rescue_min_marker_perimeter_rate_;
    rescue_params->errorCorrectionRate = static_cast<float>(block_rescue_error_correction_rate_);
    rescue_params->minDistanceToBorder = block_rescue_min_distance_to_border_;

    std::vector<int> rescue_ids;
    std::vector<std::vector<cv::Point2f>> rescue_corners;
    std::vector<std::vector<cv::Point2f>> rescue_rejected;
    cv::aruco::detectMarkers(
      padded_rescue,
      dictionary_,
      rescue_corners,
      rescue_ids,
      rescue_params,
      rescue_rejected);

    // Minimum pixel distance between two detections of the same ID to be considered distinct instances.
    constexpr float kMinInstanceDistSq = 80.0f * 80.0f;

    int recovered_count = 0;
    for (std::size_t i = 0; i < rescue_ids.size(); ++i) {
      const int id = rescue_ids[i];
      if (!block_ids_.count(id)) {
        continue;
      }

      // Compute centre of this rescue detection (in padded coords → subtract pad later)
      auto &rc = rescue_corners[i];
      cv::Point2f rescue_centre(0.f, 0.f);
      for (auto &pt : rc) rescue_centre += pt;
      rescue_centre *= 0.25f;
      rescue_centre.x -= static_cast<float>(pad_px);
      rescue_centre.y -= static_cast<float>(pad_px);

      // Skip if a detection of the same ID already exists nearby in the merged set.
      bool duplicate = false;
      for (std::size_t j = 0; j < ids.size(); ++j) {
        if (ids[j] != id) continue;
        auto &ec = corners[j];
        cv::Point2f existing_centre(0.f, 0.f);
        for (auto &pt : ec) existing_centre += pt;
        existing_centre *= 0.25f;
        float dx = rescue_centre.x - existing_centre.x;
        float dy = rescue_centre.y - existing_centre.y;
        if (dx * dx + dy * dy < kMinInstanceDistSq) { duplicate = true; break; }
      }
      if (duplicate) continue;

      std::vector<cv::Point2f> adjusted = rescue_corners[i];
      for (auto &pt : adjusted) {
        pt.x -= static_cast<float>(pad_px);
        pt.y -= static_cast<float>(pad_px);
        pt.x = std::clamp(pt.x, 0.0f, static_cast<float>(gray.cols - 1));
        pt.y = std::clamp(pt.y, 0.0f, static_cast<float>(gray.rows - 1));
      }

      ids.push_back(id);
      corners.push_back(adjusted);
      recovered_count++;
    }

    if (recovered_count > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Recovered %d block marker(s) in border rescue pass", recovered_count);
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

  bool pixelToMapXY(
    const cv::Point2f &uv_px,
    const cv::Matx33d &R_map_camera,
    const cv::Vec3d &t_map_camera,
    double z_plane,
    cv::Point2d &out_xy) const
  {
    std::vector<cv::Point2f> src = {uv_px};
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(src, undistorted, camera_matrix_, dist_coeffs_);

    const cv::Vec3d ray_camera(undistorted[0].x, undistorted[0].y, 1.0);
    const cv::Vec3d ray_map = R_map_camera * ray_camera;

    const double denom = ray_map[2];
    if (std::abs(denom) < 1e-9) {
      return false;
    }

    const double scale = (z_plane - t_map_camera[2]) / denom;
    if (scale <= 0.0) {
      return false;
    }

    const cv::Vec3d p_map = t_map_camera + scale * ray_map;
    out_xy = cv::Point2d(p_map[0], p_map[1]);
    return true;
  }

  bool yawFromMarkerCornersMap(
    const std::vector<cv::Point2f> &px_corners,
    const cv::Matx33d &R_map_camera,
    const cv::Vec3d &t_map_camera,
    double z_plane,
    double &yaw_out) const
  {
    if (px_corners.size() != 4) {
      return false;
    }

    cv::Point2d p0;
    cv::Point2d p1;
    if (!pixelToMapXY(px_corners[0], R_map_camera, t_map_camera, z_plane, p0)) {
      return false;
    }
    if (!pixelToMapXY(px_corners[1], R_map_camera, t_map_camera, z_plane, p1)) {
      return false;
    }

    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    if ((vx * vx + vy * vy) < 1e-12) {
      return false;
    }

    yaw_out = std::atan2(vy, vx);
    return true;
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

    std::lock_guard<std::mutex> lock(debug_frame_mutex_);
    debug_frame_ = image.clone();
  }

  void displayTick()
  {
    cv::Mat image;
    {
      std::lock_guard<std::mutex> lock(debug_frame_mutex_);
      if (debug_frame_.empty()) {
        return;
      }
      image = debug_frame_.clone();
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
  rclcpp::TimerBase::SharedPtr display_timer_;

  // Camera / calibration
  cv::VideoCapture cap_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Background camera processing
  std::atomic<bool> running_{false};
  std::thread processing_thread_;

  // Debug display is kept on the ROS executor thread for OpenCV GUI stability.
  std::mutex debug_frame_mutex_;
  cv::Mat debug_frame_;

  // ArUco
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  // Marker geometry
  cv::Mat obj_points_table_;
  cv::Mat obj_points_robot_;
  cv::Mat obj_points_enemy_robot_;
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

  // Snapshot-based pose stabilization
  bool have_golden_pose_{false};
  cv::Vec3d golden_rvec_{0.0, 0.0, 0.0};
  cv::Vec3d golden_tvec_{0.0, 0.0, 0.0};
  bool have_fallback_pose_{false};
  cv::Vec3d fallback_rvec_{0.0, 0.0, 0.0};
  cv::Vec3d fallback_tvec_{0.0, 0.0, 0.0};
  bool have_active_pose_{false};
  cv::Vec3d active_rvec_{0.0, 0.0, 0.0};
  cv::Vec3d active_tvec_{0.0, 0.0, 0.0};
  std::chrono::steady_clock::time_point last_pose_update_time_{};
  double pose_update_interval_s_{5.0};
  
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
  int camera_backlight_compensation_{-1};
  int camera_autofocus_{-1};
  int camera_focus_absolute_{-1};
  int camera_exposure_time_absolute_{-1};
  bool enable_clahe_preprocessing_{false};
  double clahe_clip_limit_{3.0};
  int clahe_tile_size_{16};
  bool debug_view_{true};

  int robot_marker_id_{1};
  int enemy_robot_marker_id_{6};
  int min_table_markers_{1};

  double table_marker_length_m_{0.10};
  double robot_marker_length_m_{0.07};
  double enemy_robot_marker_length_m_{0.07};
  double enemy_robot_size_x_m_{0.35};
  double enemy_robot_size_y_m_{0.35};
  double enemy_robot_obstacle_z_m_{0.05};
  double block_marker_length_m_{0.03};
  double block_size_x_m_{0.15};
  double block_size_y_m_{0.05};
  double block_center_z_m_{0.03};

  double base_link_to_aruco_x_{0.0};
  double base_link_to_aruco_y_{0.0};
  double base_link_to_aruco_z_{0.0};
  double base_link_to_aruco_yaw_{0.0};

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

  bool enable_block_marker_border_rescue_pass_{true};
  double block_rescue_pad_ratio_{0.06};
  double block_rescue_clahe_clip_limit_{3.0};
  int block_rescue_adaptive_thresh_win_size_max_{181};
  int block_rescue_adaptive_thresh_win_size_step_{30};
  double block_rescue_min_marker_perimeter_rate_{0.003};
  double block_rescue_error_correction_rate_{0.6};
  int block_rescue_min_distance_to_border_{0};

  // Bird's-eye-view block rescue
  bool enable_bev_block_rescue_pass_{true};
  double bev_block_rescue_scale_px_per_m_{1000.0};
  cv::Mat bev_H_;      // image pixel → BEV pixel (computed when pose is valid)
  cv::Mat bev_H_inv_;  // BEV pixel → image pixel
  bool bev_H_valid_{false};
};

int main(int argc, char **argv)
{
  // Inject the installed camera_global_map.yaml as default params so the node
  // works with plain `ros2 run` without requiring --params-file every time.
  // Any user-provided --ros-args (including --params-file) are appended after
  // the default and will therefore override it (ROS2 last-write-wins).
  std::vector<std::string> args_storage;
  args_storage.reserve(argc + 4);
  args_storage.push_back(argv[0]);

  // Find where --ros-args begins in the original argv, if at all.
  int ros_args_start = -1;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--ros-args") {
      ros_args_start = i;
      break;
    }
  }

  bool injected = false;
  try {
    const auto pkg_share =
      ament_index_cpp::get_package_share_directory("camera_localization");
    const auto default_yaml = pkg_share + "/config/camera_global_map.yaml";
    args_storage.push_back("--ros-args");
    args_storage.push_back("--params-file");
    args_storage.push_back(default_yaml);
    // Append the original ROS args (everything after --ros-args) so they
    // override individual parameters from the default file.
    if (ros_args_start >= 0) {
      for (int i = ros_args_start + 1; i < argc; ++i) {
        args_storage.push_back(argv[i]);
      }
    }
    injected = true;
  } catch (const std::exception &) {
    // Package share not found (e.g. not installed); fall back to original args.
    for (int i = 1; i < argc; ++i) {
      args_storage.push_back(argv[i]);
    }
  }

  std::vector<char *> new_argv;
  new_argv.reserve(args_storage.size());
  for (auto & s : args_storage) {
    new_argv.push_back(const_cast<char *>(s.c_str()));
  }
  int new_argc = static_cast<int>(new_argv.size());

  rclcpp::init(new_argc, new_argv.data());
  if (injected) {
    RCLCPP_INFO(
      rclcpp::get_logger("global_localization_node"),
      "Loaded default params from camera_global_map.yaml");
  }
  rclcpp::spin(std::make_shared<CameraLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
