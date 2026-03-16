#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stdexcept>

class CameraLocalizationNode : public rclcpp::Node
{
public:
  CameraLocalizationNode()
  : Node("camera_localization_node")
  {
    // ----------------------------
    // Parameters
    // ----------------------------
    device_ = declare_parameter<std::string>("device", "/dev/eurobot2026-ELPcamera");
    width_ = declare_parameter<int>("width", 3840);
    height_ = declare_parameter<int>("height", 2160);
    fps_ = declare_parameter<int>("fps", 30);
    fourcc_ = declare_parameter<std::string>("fourcc", "MJPG");
    calibration_file_ = declare_parameter<std::string>(
      "calibration_file",
      "camera_calibration/real/3840_2160_ELM12MP.yml");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/camera/global_pose");
    debug_view_ = declare_parameter<bool>("debug_view", true);

    // Marker IDs / sizes
    robot_marker_id_ = declare_parameter<int>("robot_marker_id", 1);
    table_marker_length_m_ = declare_parameter<double>("table_marker_length_m", 0.10);
    robot_marker_length_m_ = declare_parameter<double>("robot_marker_length_m", 0.07);

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

    // ----------------------------
    // Publisher
    // ----------------------------
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 10);

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

    RCLCPP_INFO(get_logger(), "camera_localization_node started");
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

    RCLCPP_INFO(get_logger(), "Opened camera %s", device_.c_str());
  }

  void loadCalibration()
  {
    cv::FileStorage fs(calibration_file_, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      throw std::runtime_error("Could not open calibration file: " + calibration_file_);
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

    RCLCPP_INFO(get_logger(), "Loaded calibration from %s", calibration_file_.c_str());
  }

  void initMarkerGeometry()
  {
    obj_points_table_ = makeSquareObjectPoints(table_marker_length_m_);
    obj_points_robot_ = makeSquareObjectPoints(robot_marker_length_m_);
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
    table_ids_ = {20, 21, 22, 23};

    // Assumes marker axes are aligned with map axes.
    const auto R_identity = cv::Matx33d::eye();
    table_pose_global_[20] = PoseGlobal{R_identity, cv::Vec3d(0.6, 1.4, 0.0)};
    table_pose_global_[21] = PoseGlobal{R_identity, cv::Vec3d(2.4, 1.4, 0.0)};
    table_pose_global_[22] = PoseGlobal{R_identity, cv::Vec3d(0.6, 0.6, 0.0)};
    table_pose_global_[23] = PoseGlobal{R_identity, cv::Vec3d(2.4, 0.6, 0.0)};
  }

  void initDetector()
  {
    detector_params_ = cv::aruco::DetectorParameters();
    detector_params_.perspectiveRemovePixelPerCell = 8;
    detector_params_.adaptiveThreshWinSizeMin = 3;
    detector_params_.adaptiveThreshWinSizeMax = 23;
    detector_params_.adaptiveThreshWinSizeStep = 3;
    detector_params_.adaptiveThreshConstant = 7;
    detector_params_.minMarkerPerimeterRate = 0.005;
    detector_params_.maxMarkerPerimeterRate = 4.0;
    detector_params_.errorCorrectionRate = 0.2f;
    detector_params_.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_ = cv::aruco::ArucoDetector(dictionary_, detector_params_);
  }

  void cameraTick()
  {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty camera frame");
      return;
    }

    processFrame(frame);
  }

  void processFrame(const cv::Mat &frame)
  {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    detector_.detectMarkers(gray, corners, ids, rejected);

    cv::Mat debug_image;
    if (debug_view_) {
      debug_image = frame.clone();
      if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(debug_image, corners, ids);
      }
    }

    if (ids.empty()) {
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
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Not enough table markers for camera pose");
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
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera solvePnP failed");
      showDebug(debug_image);
      return;
    }

    have_camera_pose_guess_ = true;
    last_rvec_camera_map_ = rvec_camera_map;
    last_tvec_camera_map_ = tvec_camera_map;

    cv::Mat Rcv_camera_map;
    cv::Rodrigues(rvec_camera_map, Rcv_camera_map);
    cv::Matx33d R_camera_map(Rcv_camera_map);

    // Invert to get map <- camera
    const cv::Matx33d R_map_camera = R_camera_map.t();
    const cv::Vec3d t_map_camera = -(R_map_camera * tvec_camera_map);

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

    publishPose(t_map_base, yaw_map_base);

    if (debug_view_) {
      drawAxes(debug_image, rvec_camera_map, tvec_camera_map, 0.25f);
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
  rclcpp::TimerBase::SharedPtr timer_;

  // Camera / calibration
  cv::VideoCapture cap_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // ArUco
  cv::aruco::DetectorParameters detector_params_;
  cv::aruco::Dictionary dictionary_;
  cv::aruco::ArucoDetector detector_;

  // Marker geometry
  cv::Mat obj_points_table_;
  cv::Mat obj_points_robot_;

  // Known map markers
  std::unordered_set<int> table_ids_;
  std::unordered_map<int, PoseGlobal> table_pose_global_;

  // Camera pose guess for faster/stabler solvePnP
  bool have_camera_pose_guess_{false};
  cv::Vec3d last_rvec_camera_map_{0.0, 0.0, 0.0};
  cv::Vec3d last_tvec_camera_map_{0.0, 0.0, 0.0};

  // Parameters
  std::string device_;
  std::string fourcc_;
  std::string calibration_file_;
  std::string map_frame_;
  std::string pose_topic_;
  int width_{0};
  int height_{0};
  int fps_{30};
  bool debug_view_{true};

  int robot_marker_id_{1};
  int min_table_markers_{1};

  double table_marker_length_m_{0.10};
  double robot_marker_length_m_{0.07};

  double marker_to_base_x_{0.0};
  double marker_to_base_y_{0.0};
  double marker_to_base_z_{0.0};
  double marker_to_base_yaw_{0.0};

  double position_variance_x_{0.01};
  double position_variance_y_{0.01};
  double yaw_variance_{0.03};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}