////////////////////////////////////////////////////////////////////////////////////////////////
// Convention for transforms : T_A_B: takes a point in frame B and expresses it in frame A.
//                            -> X_camera = T_camera_global * X_global
//                            -> X_global = T_global_camera * X_camera
////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////
// ENTITY MESSAGE FORMAT (entities/all topic):
// Float32MultiArray.data = [n_entities, entity_0_data, entity_1_data, ...]
// where each entity_data is 7 floats:
//   [track_id, entity_type, class_id, x_m, y_m, z_m, yaw_rad]
//   entity_type: 0=block, 1=robot
//   class_id: 0=our_team, 1=opponent
//   positions in meters, yaw in radians [-π, +π]
////////////////////////////////////////////////////////////////////////////////////////////////

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>
#include <array>
#include <filesystem>   // C++17
#include <numeric> // iota
#include <cmath> 
#include <unordered_set> //fast check if an ID is in a group
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>


class PerceptionNode : public rclcpp::Node {
public: 
    PerceptionNode() : rclcpp::Node("perception_node") {
    
    // Camera settings as parameters (so you can change later)
    const auto camera_index = declare_parameter<int>("camera_index", 0);
    const auto camera_path = declare_parameter<std::string>("camera_path", "/dev/video0");
    device_ = camera_path.empty() ? "/dev/video" + std::to_string(camera_index) : camera_path;
    width_  = declare_parameter<int>("width", 3840);
    height_ = declare_parameter<int>("height", 2160);
    fps_    = declare_parameter<int>("fps", 30);
    // fourcc_ = declare_parameter<std::string>("fourcc", "YUYV");
    fourcc_ = declare_parameter<std::string>("fourcc", "MJPG");

    // Open camera ONCE
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        throw std::runtime_error("Failed to open camera " + device_);
    }

    // Create timer to time how many times we do detection. Goal is to be exact as the cameras fps
    const int period_ms = std::max(1, static_cast<int>(1000.0 / fps_));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&PerceptionNode::cameraTick, this)
    );


    // Combined publisher for all entities (blocks + robots)
    // We'll publish PoseArray in "entities/poses" with metadata in "entities/meta"
    // OR better: use a single custom message
    entities_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("entities/all", 10); // 10 is queue size


    // Apply settings ONCE
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);
    
    // Log camera parameters
    RCLCPP_INFO(get_logger(), "Camera settings: FOURCC=%s, Resolution=%dx%d, FPS=%d, BufferSize=1",
                fourcc_.c_str(), width_, height_, fps_);
    
    // cap_.set(cv::CAP_PROP_ZOOM,         100);  // some drivers expose MJPG quality via ZOOM




    // Camera parameters
    std::string calib_file = declare_parameter<std::string>(
        "calibration_file",
        "camera_calibration/real/3840_2160_ELM12MP.yml");
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cerr << "ERROR: Could not open calibration file!" << std::endl;
    }

    int imageWidth_, imageHeight_;
    double rms_, avgReprojError_;

    fs["image_width"]  >> imageWidth_;
    fs["image_height"] >> imageHeight_;
    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;
    fs["rms"] >> rms_;
    fs["avg_reprojection_error_px"] >> avgReprojError_;

    fs.release();



    // aruco side length (only the inside)
    Tbl_markerLength_ = 0.10f;   // 10 cm for table markers
    Blc_markerLength_ = 0.03f;  // 3 cm for blocks
    Rob_markerLength_ = 0.07f;   // 7 cm for robots

    // 0 = blue (IDs 1-5), 1 = yellow (IDs 6-10)
    // Team configuration: which team are we?
    // our_team_ = declare_parameter<int>("our_team", 0);
    our_team_ = 0;


    // Ids of every element
    table_ids_ = {20, 21, 22, 23};
    block_ids_ = {36, 47};      // blue=36, yellow=47
    robot_ids_ = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};  // blue=1-5, yellow=6-10

    // Global positons of markers (who have same orientation as the global frame)
    auto R_identity = cv::Matx33d::eye();
    table_pose_global_[20] = PoseGlobal{R_identity, cv::Vec3d(0.6, 1.4, 0.0)};
    table_pose_global_[21] = PoseGlobal{R_identity, cv::Vec3d(2.4, 1.4, 0.0)};
    table_pose_global_[22] = PoseGlobal{R_identity, cv::Vec3d(0.6, 0.6, 0.0)};
    table_pose_global_[23] = PoseGlobal{R_identity, cv::Vec3d(2.4, 0.6, 0.0)};


    // Define the markers 3D coordinates
    //// 3D corners for TABLE markers
    objPointsTbl_ = cv::Mat(4, 1, CV_32FC3);
    objPointsTbl_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-Tbl_markerLength_/2.f,  Tbl_markerLength_/2.f, 0);
    objPointsTbl_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( Tbl_markerLength_/2.f,  Tbl_markerLength_/2.f, 0);
    objPointsTbl_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( Tbl_markerLength_/2.f, -Tbl_markerLength_/2.f, 0);
    objPointsTbl_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-Tbl_markerLength_/2.f, -Tbl_markerLength_/2.f, 0);

    //// 3D corners for BLOCK markers
    objPointsBlc_ = cv::Mat(4, 1, CV_32FC3);
    objPointsBlc_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-Blc_markerLength_/2.f,  Blc_markerLength_/2.f, 0.0f);
    objPointsBlc_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( Blc_markerLength_/2.f,  Blc_markerLength_/2.f, 0.0f);
    objPointsBlc_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( Blc_markerLength_/2.f, -Blc_markerLength_/2.f, 0.0f);
    objPointsBlc_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-Blc_markerLength_/2.f, -Blc_markerLength_/2.f, 0.0f);

    //// 3D corners for ROBOT markers
    objPointsRob_ = cv::Mat(4, 1, CV_32FC3);
    objPointsRob_.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-Rob_markerLength_/2.f,  Rob_markerLength_/2.f, 0.0f);
    objPointsRob_.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( Rob_markerLength_/2.f,  Rob_markerLength_/2.f, 0.0f);
    objPointsRob_.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( Rob_markerLength_/2.f, -Rob_markerLength_/2.f, 0.0f);
    objPointsRob_.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-Rob_markerLength_/2.f, -Rob_markerLength_/2.f, 0.0f);



    // parameters - todo: explore later
    // detectorParams_ = cv::aruco::DetectorParameters();

    // detectorParams_.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // detectorParams_.adaptiveThreshWinSizeMin = 3;
    // detectorParams_.adaptiveThreshWinSizeMax = 35; // helps with contrast issues
    // detectorParams_.adaptiveThreshWinSizeStep = 10;
    // detectorParams_.adaptiveThreshConstant = 7;

    // detectorParams_.minMarkerPerimeterRate = 0.01;  // allow smaller markers (try 0.003)
    // detectorParams_.maxMarkerPerimeterRate = 4.0;

    // detectorParams_.errorCorrectionRate = 0.2f;            // even 0.1 (to manage flickery detections)
    // detectorParams_.maxErroneousBitsInBorderRate = 0.05f;  // even 0.02 (to manage flickery detections)


    // detectorParams_.perspectiveRemovePixelPerCell = 4; // try 8, 10, 12
    // detectorParams_.perspectiveRemoveIgnoredMarginPerCell = 0.13;

    // --- OPTIMIZED PARAMETERS START ---
    detectorParams_ = cv::aruco::DetectorParameters::create();

    // 1. STABILITY (Stops flickering)
    // Resolution of the extracted bits. 4 is too low for 4K. 
    // Higher = more stable ID reading, but slightly slower.
    detectorParams_->perspectiveRemovePixelPerCell = 8; 
    
    // 2. DETECTION (Finds the squares)
    // Adaptive thresholding: These control how we turn the color image into black/white
    detectorParams_->adaptiveThreshWinSizeMin = 3;
    detectorParams_->adaptiveThreshWinSizeMax = 23; // Lower max helps avoid lighting gradients across big table
    detectorParams_->adaptiveThreshWinSizeStep = 3; // Smaller step = checks more window sizes = better detection
    detectorParams_->adaptiveThreshConstant = 7;    // Constant subtracted from mean. 7 is usually good.

    // 3. FILTERING (Stops "Ghosts" and noise)
    // Ignore really tiny contours (noise) or really huge ones (walls)
    detectorParams_->minMarkerPerimeterRate = 0.005; // 0.01 was maybe picking up noise. 0.02 is safer for 3cm blocks.
    detectorParams_->maxMarkerPerimeterRate = 4.0;
    
    // TIGHTER ERROR CORRECTION
    // Lower = Stricter. 0.6 is default. 0.2 is good. 
    // If ghosts persist, try 0.1, but it might make blocks harder to detect.
    detectorParams_->errorCorrectionRate = 0.2f; 
    
    // Use Subpix for better pose accuracy 
    detectorParams_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; 
    // detectorParams_.cornerRefinementWinSize = 3;  // default is 5, try 3 for tiny markers
    // detectorParams_.cornerRefinementMaxIterations = 30;
    // detectorParams_.cornerRefinementMinAccuracy = 0.1;

    // --- OPTIMIZED PARAMETERS END ---
    
    // family of aruco codes to look for
    dictionary_     = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_.c_str());
    cv::namedWindow("out",cv::WINDOW_NORMAL);
}

private:
// For each table marker ID, we will store its known positon in global using R and t from eutobot documentation
struct PoseGlobal {
    cv::Matx33d R_global_marker;  // rotation from marker to global
    cv::Vec3d   t_global_marker;  // translation of marker origin in global
};

// Represent one observation from the camera in a frame
struct Detection {
    int entity_type;  // 0=block, 1=robot
    int class_id;     // 0=our_team, 1=opponent
    cv::Vec2d xy;     // measured x,y in global meters
    double yaw;       // radians, around global Z
};

// Memory of one block or robot
struct Track {
    int track_id = -1;
    int entity_type = -1;    // 0=block, 1=robot
    int class_id = -1;       // 0=our_team, 1=opponent
    cv::Vec2d pos{0.0, 0.0}; // filtered x,y (EMA)
    double yaw = 0.0;        // filtered yaw (radians)
    int missed_frames = 0;   // how many frames was this entity not seen
};

// Function to wrap yaw angle of blocks to -pi pi and apply ema to it
double angleEma(double old_yaw, double meas_yaw) const{
    // shortest signed angular difference meas-old in [-pi, +pi]
    double diff = std::atan2(std::sin(meas_yaw - old_yaw), std::cos(meas_yaw - old_yaw)); // the antan2 does the wrapToPi
    return wrapPi(old_yaw + yaw_alpha_ * diff);
}

void cameraTick() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) return;
    processFrame(frame);
}

// Convert pixel (u,v) to global XY by intersecting camera ray with plane z=z_plane
bool pixelToGlobalXY(
    const cv::Point2f& uv_px,
    const cv::Matx33d& R_global_camera,
    const cv::Vec3d& t_global_camera,
    double z_plane,
    cv::Point2d& out_xy) const
{
    // 1) Undistort the pixel to normalized camera coordinates
    std::vector<cv::Point2f> src = { uv_px };
    std::vector<cv::Point2f> und;
    cv::undistortPoints(src, und, cameraMatrix_, distCoeffs_);

    double x = und[0].x;
    double y = und[0].y;

    // 2) Ray in camera frame (normalized coords => direction [x, y, 1])
    cv::Vec3d ray_cam(x, y, 1.0);

    // 3) Ray direction in global frame
    cv::Vec3d ray_global = R_global_camera * ray_cam;

    // 4) Camera origin in global frame is t_global_camera
    // Intersect: p = t + s * ray, with p.z = z_plane
    double denom = ray_global[2];
    if (std::abs(denom) < 1e-9) return false; // ray parallel to plane

    double s = (z_plane - t_global_camera[2]) / denom;
    if (s <= 0) return false; // intersection behind camera

    cv::Vec3d p = t_global_camera + s * ray_global;

    out_xy = cv::Point2d(p[0], p[1]);
    return true;
}

// Wrap angle to [-pi, pi]
double wrapPi(double a) const{
    return std::atan2(std::sin(a), std::cos(a));
}

// Compute yaw from marker corners by projecting 2 corners onto the table plane
// Uses corners[0] -> corners[1] direction as marker X axis
bool yawFromMarkerCornersGlobal(
    const std::vector<cv::Point2f>& px_corners,
    const cv::Matx33d& R_global_camera,
    const cv::Vec3d& t_global_camera,
    double z_plane,
    double& yaw_out) const {
    
    // make sure its an actual aruco, and check it has 4 corners
    if (px_corners.size() != 4) return false;

    // Projected global points of corner 0 and corner 1
    cv::Point2d p0, p1;
    if (!pixelToGlobalXY(px_corners[0], R_global_camera, t_global_camera, z_plane, p0)) return false;
    if (!pixelToGlobalXY(px_corners[1], R_global_camera, t_global_camera, z_plane, p1)) return false;

    // Build a 2D vector from p0 to p1
    double vx = p1.x - p0.x;
    double vy = p1.y - p0.y;

    // If it’s almost zero, corners are basically identical (bad data), so fail
    if ((vx * vx + vy * vy) < 1e-12) return false;

    yaw_out = wrapPi(std::atan2(vy, vx));
    return true;

}


// Blocks tracking function
//// dets is the list of block detections saw at this frame
void updateTracksSimple(const std::vector<Detection>& dets){

    // Mark which block tracks got matched this frame
    std::vector<bool> track_matched(tracks_.size(), false);
    
    // For each detection, find the nearest unmatched track of same class
    for (const auto& det : dets) {
        // will store which track idx is the best : -1 not found yet
        int best_t = -1;

        // start best distance as maximum allowed distance 
        double best_dist = match_gate_m_;

        // loop over exery exisisting track
        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (track_matched[t]) continue;                 // already used this track
    
            // Must match both entity type AND team
            if (tracks_[t].entity_type != det.entity_type) continue;
            if (tracks_[t].class_id != det.class_id) continue;

            // distance
            cv::Vec2d diff = det.xy - tracks_[t].pos;
            double dist = std::sqrt(diff.dot(diff));

            if (dist < best_dist) {
                best_dist = dist;
                best_t = static_cast<int>(t);
            }
        }

        // if we found a match 
        if (best_t >= 0) {
            // Update existing track using EMA smoothing
            tracks_[best_t].pos = ema_alpha_ * det.xy + (1.0 - ema_alpha_) * tracks_[best_t].pos;
            tracks_[best_t].yaw = angleEma(tracks_[best_t].yaw, det.yaw);
            tracks_[best_t].missed_frames = 0; // not msised in this frame
            track_matched[best_t] = true; // and its matched
            
        } else { // if not found a bestmatch, best_t = -1
            // No suitable track: create a new one
            Track tr;
            tr.track_id = next_track_id_++; //asign unique id
            tr.class_id = det.class_id; 
            tr.entity_type = det.entity_type;
            tr.pos = det.xy;
            tr.yaw = det.yaw;
            tr.missed_frames = 0;
            tracks_.push_back(tr); // add ot list of tracks 

            // Keep bookkeeping aligned (we added a new track)
            track_matched.push_back(true);
        }
    }

    // increment missed_frames only for tracks that were NOT matched
    for (size_t t = 0; t < tracks_.size(); ++t) {
        if (!track_matched[t]) {
            tracks_[t].missed_frames += 1;
        }
    }

    // delete dead tracks
    // note remove_if only re-aregnes the vector where the end is populated by the elements to take off
    tracks_.erase(
        std::remove_if(
            tracks_.begin(), 
            tracks_.end(),
            [&](const Track& tr){ return tr.missed_frames > max_missed_frames_; }),
        tracks_.end()
    );

}

//process frames 
void processFrame(const cv::Mat& image){
        // Mark all known blocks as "not seen this frame"
        std::unordered_set<int> seen_blocks_this_frame;

        // Convert image to gray        
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        // Boost contrast for small markers without changing ArUco params
        // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(16,16));
        // clahe->apply(gray_image, gray_image);

        // Run detection on the grayscale image
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(gray_image, dictionary_, corners, ids, detectorParams_, rejected);


        // Prepare output arrays for pose (rotation/translation) for each detected marker
        size_t nMarkers = corners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        // Estimate pose of aruco markers /(old version) - for debug onlly, to able to see the axis of the aruco - to remove in deployement hihi
        for (size_t i = 0; i < nMarkers; i++) {
            const cv::Mat& objPts = getObjPointsForId(ids[i]);
            cv::solvePnP(objPts, corners.at(i), cameraMatrix_, distCoeffs_, rvecs.at(i), tvecs.at(i), false, cv::SOLVEPNP_IPPE_SQUARE);
        }

        // // Estimate pose of aruco markers
        // for (size_t i = 0; i < nMarkers; i++) {
        //     const cv::Mat& objPts = getObjPointsForId(ids[i]);

        //     // Unlike solvePnP, this returns ALL possible mathematical solutions (usually 2)
        //     std::vector<cv::Mat> rvecs_candidates, tvecs_candidates;
        //     std::vector<double> reproj_errors;
        //     cv::solvePnPGeneric(objPts, corners.at(i), cameraMatrix_, distCoeffs_, 
        //                         rvecs_candidates, tvecs_candidates, 
        //                         false, cv::SOLVEPNP_IPPE_SQUARE, 
        //                         cv::noArray(), cv::noArray(), reproj_errors);

        //     // Figure out which solution is the "real" one.
        //     // We assume the markers are on the table, facing UP.
        //     // The Camera is looking DOWN (Standard OpenCV camera Z-axis points into the scene).
        //     // Therefore, the Marker's Z-axis must point TOWARDS the camera (Negative Z in camera frame).
        //     int best_idx = 0;
            
        //     // If we have ambiguity (more than 1 solution), check the orientation
        //     if (rvecs_candidates.size() > 1) {
        //         // Get Rotation Matrix for Solution 0
        //         cv::Mat R0;
        //         cv::Rodrigues(rvecs_candidates[0], R0);
        //         // The element at (2,2) tells us if the Z-axis is pointing away or towards the camera
        //         double z_dir0 = R0.at<double>(2, 2); 

        //         // Get Rotation Matrix for Solution 1
        //         cv::Mat R1;
        //         cv::Rodrigues(rvecs_candidates[1], R1);
        //         double z_dir1 = R1.at<double>(2, 2);

        //         // We want the solution where Z is pointing "back" at the camera (Most negative value)
        //         if (z_dir1 < z_dir0) {
        //             best_idx = 1;
        //         }
        //     }

        //     // STEP 3: Assign the best solution to our output lists
        //     rvecs.at(i) = rvecs_candidates[best_idx];
        //     tvecs.at(i) = tvecs_candidates[best_idx];
        // }


        // for debugging and testing pahse only - todo: take off
        // for (size_t i = 0; i < ids.size(); ++i) {
        // int id = ids[i];
        // if (table_ids_.count(id)) {
        //     std::cout << "[TABLE] id=" << id << " tvec=" << tvecs[i] << std::endl;
        // } else if (block_ids_.count(id)) {
        //     std::cout << "[BLOCK] id=" << id << " tvec=" << tvecs[i] << std::endl;
        // } else if (robot_ids_.count(id)) {
        //     std::cout << "[ROBOT] id=" << id << " tvec=" << tvecs[i] << std::endl;
        // } else {
        //     std::cout << "[OTHER] id=" << id << " tvec=" << tvecs[i] << std::endl;
        // }
        // }

        // std::cout<<"Number of detected markers :" << ids.size() << std::endl;
        
        // Draw on a copy (or draw on frame directly)
        float axis_len = 0.1f;
        cv::Mat imageCopy = image.clone();
        int id;
        for (size_t i = 0; i < ids.size(); ++i) {
            id = ids[i];
            if (table_ids_.count(id)) 
                axis_len = static_cast<float>(Tbl_markerLength_ * 1.5);
            else if (block_ids_.count(id)) 
                axis_len = static_cast<float>(Blc_markerLength_ * 1.5);
            else if (robot_ids_.count(id))
                axis_len = static_cast<float>(Rob_markerLength_ * 1.5);
                
            cv::drawFrameAxes(imageCopy, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], axis_len, 2);
        }

        // Transforms 
        // Create two empty lists : 
        std::vector<cv::Point3f> objPts_global; //3D points in global (meters)
        std::vector<cv::Point2f> imgPts;        //2D pixel points (px)

        // Reserve memory for efficiency (times 4 bcs we have 4 corners - we are storing the coords of each corner)
        objPts_global.reserve(4 * ids.size());
        imgPts.reserve(4 * ids.size());

        // Loop over each detected marker
        for (size_t i = 0; i < ids.size(); ++i) {
            int id = ids[i];

            // skip markers that are not the table's
            if (!table_ids_.count(id)) {
                continue;
            }

            // Get global corners of this marker and if its not a valid marker or it pose is not known, skip it (that why we have the if)
            std::array<cv::Point3f, 4> corners_global;
            if (!getTableMarkerCornersGlobal(id, corners_global)) {
                continue;
            }

            // We push 4 (3D global corner, 2D pixel corner) pairs - because thats what PnP needs
            for (int k = 0; k < 4; ++k) {
                objPts_global.push_back(corners_global[k]);
                imgPts.push_back(corners[i][k]);
            }
        }

        // We need at least 4 points for PnP (1 marker gives 4 corners), do we have it ?
        bool have_table_pose_this_frame = (objPts_global.size() >= 4);

        // These are the outputs of PnP
        cv::Vec3d rvec_camera_global, tvec_camera_global;
        cv::Matx33d R_camera_global;

        if (have_table_pose_this_frame) {
            bool use_guess = have_T_camera_global_; // do we use the guess from previous R and t?
            // Use last frame as initial guess if its available (makes it more efficient and faster)
            if (use_guess) {
                rvec_camera_global = last_rvec_camera_global_;
                tvec_camera_global = last_tvec_camera_global_;
            }

            // Solve the PnP problem : Solve: X_camera = R_camera_global * X_global + t_camera_global
            bool ok = cv::solvePnP(
                objPts_global,
                imgPts,
                cameraMatrix_,
                distCoeffs_,
                rvec_camera_global,
                tvec_camera_global,
                use_guess,
                cv::SOLVEPNP_ITERATIVE // todo: explore later
            );

            // If solvePnP succeeded, store it
            if (ok) {
                // Convert rvec to R
                // note : OpenCV stores rotation as a 3-number axis-angle vector (rvec)
                //        Rodrigues() converts it to a 3x3 rotation matrix R
                cv::Mat Rcv;
                cv::Rodrigues(rvec_camera_global, Rcv);
                R_camera_global = cv::Matx33d(Rcv);

                // Save this for the next frame PnP (again for effciency and speed yohoooo)
                have_T_camera_global_ = true;
                last_rvec_camera_global_ = rvec_camera_global;
                last_tvec_camera_global_ = tvec_camera_global;

            } else {
                // if it fails
                have_T_camera_global_ = false;
            }

            if (have_T_camera_global_) {
                // Invert to get T_global_camera (and print for debbuging for now)
                cv::Matx33d R_global_camera = R_camera_global.t(); // transpose
                cv::Vec3d   t_global_camera = -(R_global_camera * tvec_camera_global); // opssite of the positoon (classic for transforms)

                // vector to store all detections 
                std::vector<Detection> dets;
                dets.reserve(ids.size());


                // Process each detected marker
                for (size_t i = 0; i < ids.size(); ++i) {
                    int class_id = classFromArucoId(ids[i]);
                    if (class_id < 0) continue; // not a block or robot

                    // find entity we are detecting 
                    int entity_type = entityTypeFromArucoId(ids[i]);
                    if (entity_type < 0) continue; //skip if none 
                    
                    // Use appropriate z-plane based on entity type
                    double z_plane = (entity_type == 0) ? 0.03 : 0.43;  // 3cm for blocks, 430mm for robots

                    // Compute center pixel of the detected block marker (average of 4 corners)
                    cv::Point2f center_px = 0.25f * (corners[i][0] + corners[i][1] + 
                                                    corners[i][2] + corners[i][3]);

                    // Intersect pixel ray with table plane
                    cv::Point2d xy_global;
                    if (!pixelToGlobalXY(center_px, R_global_camera, t_global_camera, z_plane, xy_global)) {
                        continue;
                    }

                    // 2) Orientation (yaw) for this block only
                    double yaw;
                    if (!yawFromMarkerCornersGlobal(corners[i], R_global_camera, t_global_camera, z_plane, yaw)) {
                        continue;
                    }

                    // Store this 
                    Detection det;
                    det.entity_type = entity_type;
                    det.class_id = class_id;
                    det.xy = cv::Vec2d(xy_global.x, xy_global.y);
                    det.yaw = yaw;
                    dets.push_back(det);

                }

                // Update the multi-entity tracker and publish
                updateTracksSimple(dets);
                publishTracks();

                /////////////////////////////////// debuging 
                // Draw global frame origin on image for debugging
                std::vector<cv::Point3f> global_axes_pts = {
                    {0.f, 0.f, 0.f},  // origin
                    {0.3f, 0.f, 0.f}, // X axis (red)   - 30cm
                    {0.f, 0.3f, 0.f}, // Y axis (green) - 30cm
                    {0.f, 0.f, 0.3f}  // Z axis (blue)  - 30cm
                };

                std::vector<cv::Point2f> global_axes_projected;
                cv::projectPoints(global_axes_pts, rvec_camera_global, tvec_camera_global,
                                cameraMatrix_, distCoeffs_, global_axes_projected);

                // Draw the axes on imageCopy
                cv::arrowedLine(imageCopy, global_axes_projected[0], global_axes_projected[1], cv::Scalar(0,0,255),   4); // X red
                cv::arrowedLine(imageCopy, global_axes_projected[0], global_axes_projected[2], cv::Scalar(0,255,0),   4); // Y green
                cv::arrowedLine(imageCopy, global_axes_projected[0], global_axes_projected[3], cv::Scalar(255,0,0),   4); // Z blue
                cv::circle(imageCopy, global_axes_projected[0], 8, cv::Scalar(0,255,255), -1); // origin dot yellow


                std::vector<cv::Point3f> pt_to_draw = {{1.0f, 0.725f, 0.0f}};
                // std::vector<cv::Point3f> pt_to_draw = {{0.825f, 0.325f, 0.03f}};
                std::vector<cv::Point2f> pt_projected;
                cv::projectPoints(pt_to_draw, rvec_camera_global, tvec_camera_global,
                                cameraMatrix_, distCoeffs_, pt_projected);

                cv::circle(imageCopy, pt_projected[0], 10, cv::Scalar(0,255,255), -1);


                /////////////////////////////////// debuging 

            }

            
        } else {
            // if we dont have a pose for this marker
            have_T_camera_global_ = false;

        }

        // Draw results
        // cv::imshow("out original", imageCopy);
        
        // resize image to be able to see it on my monitor 
        cv::Mat imageCopy_resized;
        cv::Size new_size(1250, 960);
        cv::resize(imageCopy, imageCopy_resized, new_size); 
        cv::imshow("out resized", imageCopy_resized);

        // 1ms wait -> continuous feed. Quit on 'q' or ESC.
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            // close node cleanly
            rclcpp::shutdown();
        }

    }

// Determine if ArUco belongs to our team (0) or opponent (1)
// Returns -1 if not a block/robot marker
int classFromArucoId(int aruco_id) const {
    // Blocks
    if (aruco_id == 36) return (our_team_ == 0) ? 0 : 1;  // blue block
    if (aruco_id == 47) return (our_team_ == 1) ? 0 : 1;  // yellow block
    
    // Robots: blue=1-5, yellow=6-10
    if (aruco_id >= 1 && aruco_id <= 5) {
        return (our_team_ == 0) ? 0 : 1;  // blue robot
    }
    if (aruco_id >= 6 && aruco_id <= 10) {
        return (our_team_ == 1) ? 0 : 1;  // yellow robot
    }
    
    return -1;  // not a block or robot
}

// Determine entity type: 0=block, 1=robot
int entityTypeFromArucoId(int aruco_id) const {
    if (block_ids_.count(aruco_id)) return 0;  // block
    if (robot_ids_.count(aruco_id)) return 1;  // robot
    return -1;
}

// which 3D corner template to use for pose estimation, based on the marker ID
const cv::Mat& getObjPointsForId(int id) const {
    if (table_ids_.count(id)) return objPointsTbl_;
    if (block_ids_.count(id)) return objPointsBlc_;
    if (robot_ids_.count(id)) return objPointsRob_; 
    return objPointsTbl_; // default fallback
}

bool getTableMarkerCornersGlobal(int id, std::array<cv::Point3f, 4>& corners_global) const
{
    // Check if we know the position of the table marker in global (safety, cause we are supposed to know it anyway)
    auto it = table_pose_global_.find(id); //it : is an iterator, example, it points to { key = 21 , value = PoseGlobal{R, t} }
    if (it == table_pose_global_.end()) {
        return false; // we do not know this marker's global pose
    }

    // Extract the pose of this marker in global
    const PoseGlobal& pg = it->second; // note : it->first : the key (marker ID) and it->second : the value (PoseGlobal)

    // These are the 4 corners in the marker's own frame (same shape as objPointsTbl_). This tells us where the 4 corners of marker are
    std::array<cv::Point3f, 4> corners_marker = {
        cv::Point3f(-Tbl_markerLength_ / 2.f,  Tbl_markerLength_ / 2.f, 0.f),
        cv::Point3f( Tbl_markerLength_ / 2.f,  Tbl_markerLength_ / 2.f, 0.f),
        cv::Point3f( Tbl_markerLength_ / 2.f, -Tbl_markerLength_ / 2.f, 0.f),
        cv::Point3f(-Tbl_markerLength_ / 2.f, -Tbl_markerLength_ / 2.f, 0.f)
    };

    // Convert each corner from marker frame to global frame:
    // X_global = R_global_marker * X_marker + t_global_marker
    for (int k = 0; k < 4; ++k) {
        cv::Vec3d pm(corners_marker[k].x, corners_marker[k].y, corners_marker[k].z); //extarct the x y z positions of this corner : pm = [ x_marker , y_marker , z_marker ] in the makrers frame
        cv::Vec3d p_global_corner = pg.R_global_marker * pm + pg.t_global_marker; // find the global cooridnates of this point (corner)
        corners_global[k] = cv::Point3f(
            static_cast<float>(p_global_corner[0]),
            static_cast<float>(p_global_corner[1]),
            static_cast<float>(p_global_corner[2])
        );
    }

    return true;
}

void publishTracks(){
    auto now = get_clock()->now();

    // Pack everything into Float32MultiArray:
    // Format: [n_entities, track_id, entity_type, class_id, x, y, z, yaw_rad, ...]
    // Positions in meters, yaw in radians
    
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(1 + 7 * tracks_.size());  // header + 7 values per track
    
    msg.data.push_back(static_cast<float>(tracks_.size()));  // Number of entities
    
    for (const auto& tr : tracks_) {
        msg.data.push_back(static_cast<float>(tr.track_id));
        msg.data.push_back(static_cast<float>(tr.entity_type));    // 0=block, 1=robot
        msg.data.push_back(static_cast<float>(tr.class_id));       // 0=our_team, 1=opponent
        
        // Position in meters
        msg.data.push_back(static_cast<float>(tr.pos[0]));  // x (meters)
        msg.data.push_back(static_cast<float>(tr.pos[1]));  // y (meters)
        
        // Z based on entity type
        float z = (tr.entity_type == 0) ? 0.03f : 0.43f;
        msg.data.push_back(z);  // z (meters)
        
        // Yaw in radians
        msg.data.push_back(static_cast<float>(wrapPi(tr.yaw)));  // yaw (radians)
    }
    
    entities_pub_->publish(msg);
}

// --- Data members  ---
std::string                                                 topic_;
cv::Mat                                                     cameraMatrix_, distCoeffs_;
cv::VideoCapture                                            cap_;
int                                                         fps_;

double                                                      Tbl_markerLength_;
double                                                      Blc_markerLength_;
double                                                      Rob_markerLength_;  


cv::Mat                                                     objPointsTbl_;
cv::Mat                                                     objPointsBlc_;
cv::Mat                                                     objPointsRob_; 

std::unordered_set<int>                                     table_ids_;
std::unordered_set<int>                                     block_ids_;
std::unordered_set<int>                                     robot_ids_;   // empty for now, hook for later

// Known global poses of table markers
std::unordered_map<int, PoseGlobal>                         table_pose_global_;

// Camera pose estimation state (global <-> camera)
bool                                                        have_T_camera_global_ = false;
cv::Vec3d                                                   last_rvec_camera_global_{0, 0, 0};
cv::Vec3d                                                   last_tvec_camera_global_{0, 0, 0};

cv::Ptr<cv::aruco::DetectorParameters>                       detectorParams_;
cv::Ptr<cv::aruco::Dictionary>                              dictionary_;

rclcpp::TimerBase::SharedPtr                                timer_;

std::string                                                 device_;
std::string                                                 fourcc_;
int                                                         width_;
int                                                         height_;


// Combined entity publisher (blocks + robots)
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr      entities_pub_;

// All curret known blocks
std::vector<Track>                                          tracks_;

// every new blocks new id 
int                                                         next_track_id_ = 0;

// max distance to match detection to a track
double                                                      match_gate_m_ = 0.02; 

// amount of frames to keep flickered frames
// keep alive for flicker
int                                                         max_missed_frames_ = 5;   

const double                                                ema_alpha_ = 0.5; // EMA constant for filtering - to tune later 
double                                                      yaw_alpha_ = 0.5;   // yaw smoothing, smaller = more stable

int                                                         our_team_ = 0;  // 0=blue, 1=yellow - set which team we are


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
