#pragma once

#include "vineslam_ros.hpp"

namespace vineslam
{
class LocalizationNode : public VineSLAM_ros
{
public:
  // Class constructor that
  // - Initialize the ROS node
  // - Define the publish and subscribe topics
  LocalizationNode();

  // Class destructor - saves the map to an output xml file
  ~LocalizationNode();

private:
  // Parameters loader
  void loadParameters(Parameters& params);

  // Load topological map
  void loadMaps();

  // Get static transformations
  void getTfs();
  void getTfsFromParameters();

  // Runtime execution routines
  void init();
  void loop();
  void loopOnce();
  void process();

  // Publish the tfs exported by the localization node
  void broadcastTfsWithTopic();
  void broadcastTfs();

  // Initialization functions
  void initializeLocalizationSrv(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
  void initializeLocalization();
  void initializeFromFile(const std::string& path);
  void setupInitializationMarker();

  // Interactive marker for initialization callback functions
  void iMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void iMenuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  // Interactive marker for initialization variables
  interactive_markers::MenuHandler im_menu_handler_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

  // ROS subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<ublox_msgs::msg::NavRELPOSNED9>::SharedPtr gps_heading_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_subscriber_;

  // ROS services
  rclcpp::Service<vineslam_ros::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr initialize_localization_srv_;

  // Localization error counter
  uint32_t localization_error_counter_;
  uint8_t min_precision_value_;
};

}  // namespace vineslam
