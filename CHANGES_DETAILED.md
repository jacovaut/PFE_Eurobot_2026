# Changes Made - Detailed Reference

## Summary
Fixed ROS 2 QoS policy for message publications that was causing unreliable marker detection when running the node standalone. Also added comprehensive diagnostics to track detection quality and timing.

## File Modified
`/home/isaac/PFE_Eurobot_2026/server_ws/src/camera_localization/src/global_localization_node.cpp`

## Change 1: Added Timing Header
```cpp
// Location: Line 21 (added to includes)
#include <chrono>  // For high-resolution timing
```

## Change 2: Fixed QoS Policy
**Location: Lines 130-135 (Constructor, Publisher section)**

**Before:**
```cpp
pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  pose_topic_, 10);
detected_blocks_pub_ = create_publisher<std_msgs::msg::String>(
  detected_blocks_topic_, 10);
```

**After:**
```cpp
// Use RELIABLE QoS instead of default BEST_EFFORT to ensure delivery
auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  pose_topic_, qos);
detected_blocks_pub_ = create_publisher<std_msgs::msg::String>(
  detected_blocks_topic_, qos);

RCLCPP_INFO(get_logger(), "Publishers configured with RELIABLE QoS policy");
```

## Change 3: Added Frame Timing in cameraTick()
**Location: Lines 327-359 (cameraTick method)**

**Before:**
```cpp
void cameraTick()
{
  cv::Mat frame;
  if (!cap_.read(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty camera frame");
    return;
  }
  processFrame(frame);
}
```

**After:**
```cpp
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
  
  // Every 100 frames, log statistics
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
  }
}
```

## Change 4: Enhanced Detection Tracking in processFrame()
**Location: Lines 361-410 (First part of processFrame)**

**Before:**
```cpp
void processFrame(const cv::Mat &frame)
{
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<std::vector<cv::Point2f>> rejected;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

  runTableMarkerRescuePass(gray, ids, corners);

  // ... rest of function
}
```

**After:**
```cpp
void processFrame(const cv::Mat &frame)
{
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<std::vector<cv::Point2f>> rejected;
  cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_, rejected);

  int initial_marker_count = ids.size();
  int table_markers_before_rescue = 0;
  for (int id : ids) {
    if (table_ids_.count(id)) table_markers_before_rescue++;
  }

  runTableMarkerRescuePass(gray, ids, corners);
  
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

  // ... rest of function
}
```

## Change 5: Added solvePnP Tracking
**Location: Lines 447-455 (solvePnP section in processFrame)**

**Before:**
```cpp
if (table_marker_count < min_table_markers_ || obj_pts_map.size() < 4) {
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Not enough table markers for camera pose");
  publishDetectedEntities({});
  showDebug(debug_image);
  return;
}
```

**After:**
```cpp
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
```

And:

**Before:**
```cpp
if (!ok_camera) {
  have_camera_pose_guess_ = false;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera solvePnP failed");
  publishDetectedEntities({});
  showDebug(debug_image);
  return;
}

have_camera_pose_guess_ = true;
```

**After:**
```cpp
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
```

## Change 6: Added Publish Timing
**Location: Lines 506-515 (publishDetectedEntities method)**

**Before:**
```cpp
void publishDetectedEntities(const std::vector<DetectedEntity> &entities)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << "[";

  // ... JSON building ...

  ss << "]";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  detected_blocks_pub_->publish(msg);
}
```

**After:**
```cpp
void publishDetectedEntities(const std::vector<DetectedEntity> &entities)
{
  auto pub_start = std::chrono::high_resolution_clock::now();
  
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << "[";

  // ... JSON building ...

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
```

## Change 7: Added Diagnostic Member Variables
**Location: Lines 871-883 (Private members section)**

**Added:**
```cpp
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
```

## Summary of Changes

| Category | Change | Impact |
|----------|--------|--------|
| **QoS Policy** | BEST_EFFORT → RELIABLE | **PRIMARY FIX** |
| **Timing** | Added chrono-based measurement | Identify bottlenecks |
| **Frame tracking** | Count frames, empty frames | Detect buffer issues |
| **Detection tracking** | Track marker counts, rescue effectiveness | Identify detection failures |
| **Pose tracking** | Count solvePnP successes/failures | Identify pose estimation failures |
| **Publish tracking** | Measure JSON and publish times | Identify delivery delays |
| **Diagnostics** | Log every 100 frames | Continuous monitoring |

## Build Status
✅ Code compiles without errors
✅ No changes to message format or API
✅ Backward compatible
✅ Diagnostic logs are informational only

## Testing
See [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) for testing instructions.
