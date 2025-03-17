#include "../include/vineslam_ros.hpp"
#include "../include/convertions.hpp"

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Callbacks and observation functions
// --------------------------------------------------------------------------------

void VineSLAM_ros::landmarkListener(const vision_msgs::msg::Detection2DArray::SharedPtr dets)
{
}

void VineSLAM_ros::scanListener(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::PointCloud2Iterator<float> out_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(*msg, "z");

  input_data_.scan_pts_.clear();
  for (; out_x != out_x.end(); ++out_x, ++out_y, ++out_z)
  {
    if (std::isfinite(*out_x) && std::isfinite(*out_y) && std::isfinite(*out_z))
    {
      input_data_.scan_pts_.push_back(Point(*out_x, *out_y, *out_z));
    }
  }

  input_data_.received_scans_ = true;
}

void VineSLAM_ros::odomListener(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  header_ = msg->header;
  latest_odom_msg_ = *msg;

  // If it is the first iteration - initialize odometry origin
  if (init_odom_ && !init_flag_)
  {
    // Convert odometry msg to pose msg
    tf2::Quaternion q;
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);
    q.normalize();

    // Check if yaw is NaN
    auto yaw = static_cast<float>(tf2::getYaw(q));
    if (!std::isfinite(yaw))
      yaw = 0;

    init_odom_pose_ = Pose(msg->pose.pose.position.x, msg->pose.pose.position.y, 0, 0, 0, yaw);
    input_data_.p_wheel_odom_pose_ = Pose(0, 0, 0, 0, 0, 0);

    init_odom_ = false;
    return;
  }

  // Transform odometry msg to maps' referential frame
  tf2::Quaternion o2m_q;
  o2m_q.setRPY(init_odom_pose_.R_, init_odom_pose_.P_, init_odom_pose_.Y_);
  tf2::Transform odom2map(o2m_q, tf2::Vector3(init_odom_pose_.x_, init_odom_pose_.y_, init_odom_pose_.z_));

  tf2::Quaternion odom_q;
  odom_q.setX(msg->pose.pose.orientation.x);
  odom_q.setY(msg->pose.pose.orientation.y);
  odom_q.setZ(msg->pose.pose.orientation.z);
  odom_q.setW(msg->pose.pose.orientation.w);
  tf2::Transform odom_tf(odom_q, tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

  odom_tf = odom2map.inverseTimes(odom_tf);

  tf2::Vector3 trans = odom_tf.getOrigin();
  tf2::Quaternion rot = odom_tf.getRotation();

  input_data_.wheel_odom_pose_ = Pose(trans.x(), trans.y(), 0, 0, 0, static_cast<float>(tf2::getYaw(rot)));

  input_data_.received_odometry_ = true;

  // Publish input odometry source in maps' reference frame
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = params_.world_frame_id_;
  odom_msg.header.stamp = header_.stamp;
  odom_msg.pose.pose.position.x = trans.x();
  odom_msg.pose.pose.position.y = trans.y();
  odom_msg.pose.pose.position.z = 0;  // TODO: trans.z();
  odom_msg.pose.pose.orientation.x = rot.x();
  odom_msg.pose.pose.orientation.y = rot.y();
  odom_msg.pose.pose.orientation.z = rot.z();
  odom_msg.pose.pose.orientation.w = rot.w();
  odom_publisher_->publish(odom_msg);
}

void VineSLAM_ros::gpsListener(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  header_ = msg->header;

  double n, e, d;
  geodetic_converter_->geodetic2ned(msg->latitude, msg->longitude, msg->altitude, n, e, d);

  // Save pose
  input_data_.gnss_pose_.x_ = e;
  input_data_.gnss_pose_.y_ = n;
  input_data_.gnss_pose_.z_ = d;

  // Set received flag to true
  input_data_.received_gnss_ = true;

  // Publish gps pose
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = header_.stamp;
  pose_stamped.header.frame_id = params_.world_frame_id_;
  pose_stamped.pose.position.x = input_data_.gnss_pose_.x_;
  pose_stamped.pose.position.y = input_data_.gnss_pose_.y_;
  pose_stamped.pose.position.z = input_data_.gnss_pose_.z_;
  pose_stamped.pose.orientation.x = 0;
  pose_stamped.pose.orientation.y = 0;
  pose_stamped.pose.orientation.z = 0;
  pose_stamped.pose.orientation.w = 1;
  gps_pose_publisher_->publish(pose_stamped);

  if (init_gps_)
  {
    init_gps_ = false;
    input_data_.first_gnss_pose_ = *msg;
  }
}

void VineSLAM_ros::gpsArduSimpleRoverCallBack(ublox_msgs::msg::NavRELPOSNED9::SharedPtr msg)
{
  double offset = params_.heading_offset_ * M_PI / 180.0;
  input_data_.gnss_heading_ = Const::normalizeAngle(-msg->rel_pos_heading / 100000.0 * M_PI / 180.0 + offset);

  tf2::Quaternion q;
  q.setRPY(0, 0, input_data_.gnss_heading_);

  geometry_msgs::msg::PoseStamped gnss_heading_pose;
  gnss_heading_pose.header.stamp = header_.stamp;
  gnss_heading_pose.header.frame_id = params_.world_frame_id_;
  gnss_heading_pose.pose.position.x = robot_pose_.x_;
  gnss_heading_pose.pose.position.y = robot_pose_.y_;
  gnss_heading_pose.pose.position.z = robot_pose_.z_;
  gnss_heading_pose.pose.orientation.z = q.x();
  gnss_heading_pose.pose.orientation.y = q.y();
  gnss_heading_pose.pose.orientation.z = q.z();
  gnss_heading_pose.pose.orientation.w = q.w();

  gps_heading_publisher_->publish(gnss_heading_pose);

  init_gps_heading_ = false;
}

void VineSLAM_ros::imuListener(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  // Transform the orientation
  double roll = +(msg->vector.x - params_.imu_roll_offset_);
  double pitch = -(msg->vector.y - params_.imu_pitch_offset_);

  // Save pose
  input_data_.imu_pose_.x_ = 0;
  input_data_.imu_pose_.y_ = 0;
  input_data_.imu_pose_.z_ = 0;
  input_data_.imu_pose_.R_ = Const::normalizeAngle(roll);
  input_data_.imu_pose_.P_ = Const::normalizeAngle(pitch);
  input_data_.imu_pose_.Y_ = Const::normalizeAngle(robot_pose_.Y_);

  tf2::Quaternion imu_quat;
  imu_quat.setRPY(input_data_.imu_pose_.R_, input_data_.imu_pose_.P_, robot_pose_.Y_);

  // Publish pose
  geometry_msgs::msg::PoseStamped mag_pose;
  mag_pose.header.stamp = header_.stamp;
  mag_pose.header.frame_id = params_.world_frame_id_;
  mag_pose.pose.position.x = robot_pose_.x_;
  mag_pose.pose.position.y = robot_pose_.y_;
  mag_pose.pose.position.z = robot_pose_.z_;
  mag_pose.pose.orientation = tf2::toMsg(imu_quat);
  mag_orientation_publisher_->publish(mag_pose);
}

void VineSLAM_ros::imuDataListener(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  rclcpp::Time c_imu_observation_timestamp = msg->header.stamp;

  if (init_flag_)
  {
    return;
  }
  else if (!init_flag_ && init_gyro_)
  {
    p_imu_observation_timestamp_ = c_imu_observation_timestamp;
    input_data_.imu_data_pose_ = robot_pose_;
    input_data_.p_imu_data_pose_ = robot_pose_;
    init_gyro_ = false;
    return;
  }

  // Calculate interval between observations
  double d = (c_imu_observation_timestamp - p_imu_observation_timestamp_).seconds();

  // Transform the orientation
  tf2::Matrix3x3 imu_to_base_rotation(input_data_.base2imu_quat_);
  tf2::Vector3 angular_velocity_imu(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  tf2::Vector3 angular_velocity_base = imu_to_base_rotation * angular_velocity_imu;

  // Save pose
  input_data_.imu_data_pose_.x_ = 0;
  input_data_.imu_data_pose_.y_ = 0;
  input_data_.imu_data_pose_.z_ = 0;
  input_data_.imu_data_pose_.R_ += angular_velocity_base.x() * d;
  input_data_.imu_data_pose_.P_ += angular_velocity_base.y() * d;
  input_data_.imu_data_pose_.Y_ += angular_velocity_base.z() * d;

  // Normalize angles
  input_data_.imu_data_pose_.R_ = Const::normalizeAngle(input_data_.imu_data_pose_.R_);
  input_data_.imu_data_pose_.P_ = Const::normalizeAngle(input_data_.imu_data_pose_.P_);
  input_data_.imu_data_pose_.Y_ = Const::normalizeAngle(input_data_.imu_data_pose_.Y_);

  tf2::Quaternion gyro_quat;
  gyro_quat.setRPY(input_data_.imu_data_pose_.R_, input_data_.imu_data_pose_.P_, input_data_.imu_data_pose_.Y_);

  // Publish pose
  geometry_msgs::msg::PoseStamped gyro_pose;
  gyro_pose.header.stamp = header_.stamp;
  gyro_pose.header.frame_id = params_.world_frame_id_;
  gyro_pose.pose.position.x = robot_pose_.x_;
  gyro_pose.pose.position.y = robot_pose_.y_;
  gyro_pose.pose.position.z = robot_pose_.z_;
  gyro_pose.pose.orientation = tf2::toMsg(gyro_quat);
  gyro_orientation_publisher_->publish(gyro_pose);

  p_imu_observation_timestamp_ = c_imu_observation_timestamp;
}

void VineSLAM_ros::computeInnovation(const Pose& wheel_odom_inc, const Pose& imu_rot_inc, Pose& output_pose)
{
  // TODO (Andre Aguiar): When we use the gyro for innovation, we are only considering it for rotations...
  //                      in the future we should use the average between odometry and the gyro
  output_pose = Pose(wheel_odom_inc.x_, wheel_odom_inc.y_, 0, 0, 0, imu_rot_inc.Y_);
}

void VineSLAM_ros::getGNSSHeading()
{
  float robot_distance_traveleld = robot_pose_.norm3D();

  if (robot_distance_traveleld > 0.1 && robot_distance_traveleld < 5.0)
  {
    float phi = 0.;
    float min_dist = std::numeric_limits<float>::max();
    while (phi < 360.)
    {
      float x = std::cos(phi * DEGREE_TO_RAD) * robot_pose_.x_ - std::sin(phi * DEGREE_TO_RAD) * robot_pose_.y_;
      float y = std::sin(phi * DEGREE_TO_RAD) * robot_pose_.x_ + std::cos(phi * DEGREE_TO_RAD) * robot_pose_.y_;

      Point source(x, y, 0.);
      Point target(input_data_.gnss_pose_.x_, input_data_.gnss_pose_.y_, 0.);
      float dist = source.distanceXY(target);

      if (dist < min_dist)
      {
        min_dist = dist;
        // TODO (André Aguiar): revisit this in the future
        // params_.map_datum_head_ = Const::normalizeAngle(phi * DEGREE_TO_RAD);
      }

      phi += 0.1;
    }
  }
  else if (robot_distance_traveleld >= 5.0)
  {
    localizer_->changeGPSFlag(true);  // Set the confidence of the use of gps in the particle filter now that we have
                                      // estimated heading.
    estimate_heading_ = false;
  }
}

// ------------------------------------------------------------------------------------
// ----- ROS services
// ------------------------------------------------------------------------------------

bool VineSLAM_ros::saveMap(vineslam_ros::srv::SaveMap::Request::SharedPtr, vineslam_ros::srv::SaveMap::Response::SharedPtr)
{
  std::time_t timestamp = std::time(nullptr);

  // ----------------------------------------------------
  // ------ Export maps on xml file format
  // ----------------------------------------------------
  // Nodes
  for (size_t i = 0; i < topological_map_->graph_vertexes_.size(); i++)
  {
    if (topological_map_->map_[topological_map_->graph_vertexes_[i]].grid_map_ != nullptr)
    {
      Parameters l_params = params_;
      l_params.gridmap_width_ = topological_map_->map_[topological_map_->graph_vertexes_[i]].grid_map_->width_;
      l_params.gridmap_lenght_ = topological_map_->map_[topological_map_->graph_vertexes_[i]].grid_map_->lenght_;
      l_params.gridmap_origin_x_ = topological_map_->map_[topological_map_->graph_vertexes_[i]].grid_map_->origin_.x_;
      l_params.gridmap_origin_y_ = topological_map_->map_[topological_map_->graph_vertexes_[i]].grid_map_->origin_.y_;
      MapWriter mw(l_params, topological_map_->map_[topological_map_->graph_vertexes_[i]].index_);
      mw.writeToFile(topological_map_->map_[topological_map_->graph_vertexes_[i]].grid_map_, l_params);

      if (std::find(topological_map_->saved_nodes_.begin(), topological_map_->saved_nodes_.end(), topological_map_->map_[topological_map_->graph_vertexes_[i]].index_) == topological_map_->saved_nodes_.end())
      {
        topological_map_->saved_nodes_.push_back(topological_map_->map_[topological_map_->graph_vertexes_[i]].index_);
      }
    }
  }

  // Planes
  Parameters l_params;
  OccupancyMap tmp_grid_map(l_params, Pose(0, 0, 0, 0, 0, 0), 0, 0);
  tmp_grid_map.planes_ = topological_map_->planes_;
  MapWriter mw(l_params, params_.grid_map_files_folder_ + "planes.xml");
  mw.writeToFile(&tmp_grid_map, l_params);

  /*
  // Elevation map
  if (elevation_map_ != nullptr)
  {
    ElevationMapWriter ew(params_, timestamp);
    ew.writeToFile(elevation_map_, params_);
  }
   */

  // ----------------------------------------------------
  // ------ Export map information to xml file
  // ----------------------------------------------------
  std::ofstream xmlfile;
  xmlfile.open(params_.map_output_folder_ + "info_" + std::to_string(timestamp) + ".xml");

  // -- XML header
  xmlfile << HEADER << ENDL << ENDL;

  // -- Grid map details
  xmlfile << mw.open(INFO) << ENDL;
  xmlfile << TAB << mw.open(DATUM) << ENDL;
  xmlfile << TAB << TAB << mw.open(LATITUDE) << params_.map_datum_lat_ << mw.close(LATITUDE) << ENDL;
  xmlfile << TAB << TAB << mw.open(LONGITUDE) << params_.map_datum_long_ << mw.close(LONGITUDE) << ENDL;
  xmlfile << TAB << TAB << mw.open(ALTITUDE) << params_.map_datum_alt_ << mw.close(ALTITUDE) << ENDL;
  xmlfile << TAB << mw.close(DATUM) << ENDL;
  xmlfile << TAB << mw.open(ORIGIN) << ENDL;
  xmlfile << TAB << TAB << mw.open(Z_COORDINATE) << params_.gridmap_origin_z_ << mw.close(Z_COORDINATE) << ENDL;
  xmlfile << TAB << mw.close(ORIGIN) << ENDL;
  xmlfile << TAB << mw.open(HEIGHT) << params_.gridmap_height_ << mw.close(HEIGHT) << ENDL;
  xmlfile << TAB << mw.open(RESOLUTION) << params_.gridmap_resolution_ << mw.close(RESOLUTION) << ENDL;
  xmlfile << mw.close(INFO) << ENDL << ENDL;
  xmlfile.close();

  // ----------------------------------------------------
  // ------ Export saved nodes to a txt file
  // ----------------------------------------------------
  std::ofstream txtfile;
  txtfile.open(params_.grid_map_files_folder_ + "saved_nodes.txt");
  for (const auto& idx : topological_map_->saved_nodes_)
  {
    txtfile << idx << std::endl;
  }
  txtfile.close();

  return true;
}

}  // namespace vineslam
