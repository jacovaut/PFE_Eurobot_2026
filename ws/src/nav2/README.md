# Nav2 Package for Eurobot 2026

This package contains Nav2 configuration and launch files customized for the Eurobot 2026 robot.

## Files Structure

```
nav2/
├── config/
│   └── nav2_config.yaml          # Nav2 configuration with lidar and camera costmap layers
├── launch/
│   ├── nav2_launch.py            # Full Nav2 launch using nav2_bringup
│   └── nav2_simple_launch.py     # Simplified Nav2 launch for testing
├── package.xml                   # ROS 2 package definition
└── CMakeLists.txt               # Build configuration
```

## Configuration

The `nav2_config.yaml` includes:
- **Planner**: NavFn planner for path planning
- **Controller**: DWB local planner for velocity control
- **Behavior Tree**: Navigation behavior tree
- **Costmaps**:
  - **Global**: Camera point cloud layer + inflation
  - **Local**: Lidar obstacle layer + Camera point cloud layer + inflation

## Launch Files

### nav2_launch.py (Recommended)
Full Nav2 stack using nav2_bringup's navigation_launch.py:

```bash
ros2 launch nav2 nav2_launch.py
```

### nav2_simple_launch.py (Testing)
Simplified launch for testing individual components:

```bash
ros2 launch nav2 nav2_simple_launch.py
```

## Prerequisites

Before launching Nav2, ensure you have:

1. **Sensors running**:
   - Lidar publishing to `/scan`
   - Camera publishing PointCloud2 to `/camera/depth/points` (optional)

2. **TF transforms**:
   - `base_link` → `laser_frame`
   - `base_link` → `camera_frame` (if using camera)
   - `odom` → `base_link`

3. **Odometry**:
   - EKF publishing to `/odometry/filtered`

## Usage

1. **Build the package**:
   ```bash
   cd /path/to/workspace
   colcon build --packages-select nav2
   source install/setup.bash
   ```

2. **Launch Nav2**:
   ```bash
   ros2 launch nav2 nav2_launch.py
   ```

3. **Monitor**:
   ```bash
   ros2 topic list | grep nav2
   ros2 lifecycle list
   ```

## Customization

- Modify `config/nav2_config.yaml` for parameter tuning
- Add/remove costmap layers as needed
- Adjust velocity limits and tolerances for your robot