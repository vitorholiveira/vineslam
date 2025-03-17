#pragma once

// vineslam members
#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/visual.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/localization/localizer.hpp>
#include <vineslam/mapping/topological_map.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/elevation_map.hpp>
#include <vineslam/mapping/landmark_mapping.hpp>
#include <vineslam/mapping/lidar_mapping_omnidirectional.hpp>
#include <vineslam/mapping/lidar_mapping_livox.hpp>
#include <vineslam/interface/localization_mapping_interface.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/math/Geodetic.hpp>
#include <vineslam/map_io/map_writer.hpp>
#include <vineslam/map_io/map_parser.hpp>
#include <vineslam/map_io/topological_map_writer.hpp>
#include <vineslam/map_io/topological_map_parser.hpp>
#include <vineslam/map_io/elevation_map_writer.hpp>
#include <vineslam/map_io/elevation_map_parser.hpp>
#include <vineslam/utils/save_data.hpp>
#include <vineslam/utils/Timer.hpp>
// ----------------------------
#include <vineslam_ros/srv/save_map.hpp>
// ----------------------------

// std
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <thread>
#include <cmath>
#include <experimental/filesystem>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <ublox_msgs/msg/nav_relposned9.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

namespace vineslam
{
class VineSLAM_ros : public rclcpp::Node
{
public:
  VineSLAM_ros() = default;
  VineSLAM_ros(const std::string& node) : Node(node)
  {
  }

  // Landmark detection callback function
  void landmarkListener(const vision_msgs::msg::Detection2DArray::SharedPtr dets);

  // Scan callback function
  void scanListener(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Odometry callback function
  void odomListener(const nav_msgs::msg::Odometry::SharedPtr msg);

  // GPS callback function
  void gpsListener(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // GPS heading callback function
  void gpsArduSimpleRoverCallBack(ublox_msgs::msg::NavRELPOSNED9::SharedPtr msg);

  // IMU callback functions
  void imuListener(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void imuDataListener(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Services callbacks
  bool saveMap(vineslam_ros::srv::SaveMap::Request::SharedPtr, vineslam_ros::srv::SaveMap::Response::SharedPtr);

  // GNSS heading estimation
  void getGNSSHeading();

  // Compute the motion increment by fusing different sources of information
  void computeInnovation(const Pose& wheel_odom_inc, const Pose& imu_rot_inc, Pose& output_pose);

  // ROS node
  rclcpp::Node::SharedPtr nh_;

  // Most recent message header received
  std_msgs::msg::Header header_;

  // Tf2 broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Global thread to publish maps and other info
  void publishDenseInfo();
  // Publish semantic features map
  void publishLocalSemanticMap(const Pose& origin, const std::vector<SemanticFeature>& landmarks) const;
  void publishSemanticMap() const;
  void publishSemanticMapFromArray(const std::map<int, SemanticFeature>& landmarks) const;
  // Publish the elevation map
  void publishElevationMap() const;
  // Publish the 3D maps
  void publish3DMap();
  // Publish the topological maps
  void publishTopologicalMap();
  // Publish the 3D PCL planes
  void publish3DMap(const std::vector<Plane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
  void publish3DMap(const Pose& r_pose, const std::vector<Plane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
  // Publish the 3D PCL semi planes
  void publish3DMap(const std::vector<SemiPlane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
  void publish3DMap(const Pose& r_pose, const std::vector<SemiPlane>& planes, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);
  // Publish a 3D PCL corners map
  void publish3DMap(const std::vector<Corner>& corners, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
  void publish3DMap(const Pose& r_pose, const std::vector<Corner>& corners, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
  // Publish a 3D PCL planar features map
  void publish3DMap(const std::vector<Planar>& planars, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
  void publish3DMap(const Pose& r_pose, const std::vector<Planar>& planars, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
  // Creates a 6-DoF interactive marker
  void make6DofMarker(visualization_msgs::msg::InteractiveMarker& imarker, Pose pose, std::string marker_name);
  // Publishes a box containing the grid map
  void publishGridMapLimits() const;
  // Publishes a box containing the zone occupied by the robot
  void publishRobotBox(const Pose& robot_pose) const;

  // VineSLAM input data
  struct InputData
  {
    // Wheel odometry pose
    Pose wheel_odom_pose_;
    // Previous wheel odometry pose
    Pose p_wheel_odom_pose_;
    // GNSS pose
    Pose gnss_pose_;
    double gnss_heading_;
    sensor_msgs::msg::NavSatFix first_gnss_pose_;
    // IMU poses and transformations
    Pose imu_pose_;
    Pose imu_data_pose_;
    Pose p_imu_data_pose_;
    tf2::Quaternion base2imu_quat_;

    // LiDAR scan points
    std::vector<Point> scan_pts_;

    // Observation flags
    bool received_odometry_;
    bool received_gnss_;
    bool received_scans_;
  } input_data_;

  // ROS publishers/services
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr topological_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr elevation_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr semantic_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_box_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map3D_corners_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map3D_planars_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map3D_planes_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corners_local_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planars_local_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planes_local_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr semantic_local_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gps_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gps_heading_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mag_orientation_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gyro_orientation_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vineslam_state_publisher_;

  // Classes object members
  Parameters params_;
  Localizer* localizer_;
  ElevationMap* elevation_map_;
  OccupancyMap* grid_map_;
  TopologicalMap* topological_map_;
#if LIDAR_TYPE == 0 || LIDAR_TYPE == 1 || LIDAR_TYPE ==3
  OmnidirectionalMapper* lid_mapper_;
#elif LIDAR_TYPE == 2 || LIDAR_TYPE == 4
  LivoxMapper* lid_mapper_;
#endif
  Timer* timer_;
  Geodetic* geodetic_converter_;
  Observation obsv_;
  LocalizationMappingInterface* localization_mapping_interface_;

  // Array of poses to store and publish the robot path
  std::vector<geometry_msgs::msg::PoseStamped> path_;

  // Motion variables
  Pose init_odom_pose_;
  Pose init_gps_pose_;
  Pose robot_pose_;

  // Variable to save the latest odometry message
  nav_msgs::msg::Odometry latest_odom_msg_;

  // Timestamps
  rclcpp::Time p_imu_observation_timestamp_;

  // Satellite -> map compensation
  bool estimate_heading_;

  // Initialization flags
  bool init_flag_;
  bool init_gps_;
  bool init_gps_heading_;
  bool init_odom_;
  bool init_gyro_;

  // Logs vars
  uint32_t n_saved_logs_;
  Pose p_saved_pose_;
  std::ofstream logs_file_;
};

}  // namespace vineslam
