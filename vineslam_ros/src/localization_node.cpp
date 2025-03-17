#include "../include/localization_node.hpp"
#include "../include/convertions.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vineslam::LocalizationNode>());
  rclcpp::shutdown();

  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

LocalizationNode::LocalizationNode() : VineSLAM_ros("LocalizationNode")
{
  // Load params
  loadParameters(params_);

  // Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;
  init_gyro_ = true;

  // Initialize variables
  estimate_heading_ = false;
  localization_error_counter_ = 0;
  min_precision_value_ = 70;

  // Declare the Mappers and Localizer objects
  localizer_ = new Localizer(params_);
  geodetic_converter_ = new Geodetic(params_.map_datum_lat_, params_.map_datum_long_, params_.map_datum_alt_);
#if LIDAR_TYPE == 0
  lid_mapper_ = new OmnidirectionalMapper(params_, "Velodyne");
#elif LIDAR_TYPE == 1
  lid_mapper_ = new OmnidirectionalMapper(params_, "Robosense");
#elif LIDAR_TYPE == 2
  lid_mapper_ = new LivoxMapper(params_);
#elif LIDAR_TYPE == 3
  lid_mapper_ = new OmnidirectionalMapper(params_, "Ouster");
#elif LIDAR_TYPE == 4
  lid_mapper_ = new LivoxMapper(params_);
#endif
#if VERBOSE == 1
  timer_ = new Timer("VineSLAM subfunctions");
  localizer_->t_ = timer_;
  lid_mapper_->t_ = timer_;
#endif

  // Scan subscription
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("scan_topic", 10, std::bind(&VineSLAM_ros::scanListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // Odometry subscription
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom_topic", 10, std::bind(&VineSLAM_ros::odomListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // GPS subscription
  gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps_topic", 10, std::bind(&VineSLAM_ros::gpsListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // GPS heading subscription
  gps_heading_subscriber_ = this->create_subscription<ublox_msgs::msg::NavRELPOSNED9>("gps_heading_topic", 10, std::bind(&LocalizationNode::gpsArduSimpleRoverCallBack, this, std::placeholders::_1));
  // IMU subscriptions
  imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("imu_topic", 10, std::bind(&VineSLAM_ros::imuListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  imu_data_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_data_topic", 10, std::bind(&VineSLAM_ros::imuDataListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));

  // Feature and map publishers
  topological_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vineslam/topological_map", 10);
  map3D_planars_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vineslam/map3D/planars_map", 10);
  planars_local_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vineslam/map3D/planars_local", 10);
  // Pose publishers
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/pose", 10);
  poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("vineslam/poses", 10);
  // Input data publishers
  gps_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_gps_pose", 10);
  gps_heading_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_gps_heading", 10);
  mag_orientation_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_mag_orientation", 10);
  gyro_orientation_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_gyro_orientation", 10);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("vineslam/input_odometry", 10);
  // VineSLAM state publisher
  vineslam_state_publisher_ = this->create_publisher<std_msgs::msg::String>("vineslam_current_state", 10);

  // Localization initialization service
  initialize_localization_srv_ = this->create_service<std_srvs::srv::Empty>("vineslam/initialize_localization", std::bind(&LocalizationNode::initializeLocalizationSrv, this, std::placeholders::_1, std::placeholders::_2));

  // Static transforms
  std::cout << "\033[1;33mWaiting for static transforms...\033[0m\n" << std::flush;
  getTfs();
  std::cout << "\033[1;32mDone\033[0m\n" << std::flush;

  // Load topological map
  std::cout << "\033[1;33mAllocating map memory...\033[0m\n" << std::flush;
  loadMaps();
  std::cout << "\033[1;32mDone\033[0m\n" << std::flush;

  // Call execution thread
  std::thread th1(&LocalizationNode::loop, this);
  th1.detach();
}

void LocalizationNode::loadParameters(Parameters& params)
{
  std::string prefix = this->get_name();
  std::string param;

  // Load params
  param = "lidar_sensor_frame";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.lidar_sensor_frame_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "world_frame_id";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.world_frame_id_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "base_frame_id";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.base_frame_id_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "odom_frame_id";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.odom_frame_id_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "imu_sensor_frame";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.imu_sensor_frame_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_lidar_features";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_lidar_features_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_vertical_planes";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_vertical_planes_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_ground_plane";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_ground_plane_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_gps";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_gps_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_gps_altitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_gps_altitude_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_gps_heading";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_gps_heading_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_imu";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_imu_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "use_gyroscope";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.use_gyroscope_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "heading_offset";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.heading_offset_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "imu_roll_offset";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.imu_roll_offset_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "imu_pitch_offset";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.imu_pitch_offset_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "publish_level";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.publish_level_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "initialization_type";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.initialization_type_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.grid_map.origin.z";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_origin_z_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.grid_map.height";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_height_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.grid_map.resolution";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.gridmap_resolution_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.datum.latitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_lat_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.datum.longitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_long_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.datum.altitude";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.map_datum_alt_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "robot_dimensions.x";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_dim_x_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "robot_dimensions.y";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_dim_y_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "robot_dimensions.z";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.robot_dim_z_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.topological_map.folder";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.topological_map_folder_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "pf.n_particles";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.number_particles_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "pf.sigma_xx";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_xx_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "pf.sigma_yy";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_yy_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "pf.sigma_zz";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_zz_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "pf.sigma_RR";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_RR_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "pf.sigma_PP";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_PP_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "pf.sigma_YY";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.sigma_YY_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }

  params_.topological_map_input_file_ = params.topological_map_folder_ + "map.xml";
  params_.map_output_folder_ = params.topological_map_folder_ + "output_map/";
  params_.grid_map_files_folder_ = params.topological_map_folder_ + "nodes/";
}

void LocalizationNode::loadMaps()
{
  // Load topological map
  topological_map_ = new TopologicalMap(params_);
  TopologicalMapParser topological_map_parser(params_);
  topological_map_parser.parseFile(topological_map_);
  topological_map_->active_nodes_ref_.resize(topological_map_->graph_vertexes_.size());

  // Allocate memory for the localization and mapping interface structure
  localization_mapping_interface_ = new LocalizationMappingInterface(topological_map_->graph_vertexes_.size());

  // Initialize topological map
  topological_map_->polar2Enu(geodetic_converter_);
  topological_map_->init_function_called_ = true;
  topological_map_->init(0.0, localization_mapping_interface_, true);

  // Get indexes of nodes saved into files
  std::ifstream infile(params_.grid_map_files_folder_ + "saved_nodes.txt");
  uint32_t idx;
  std::vector<uint32_t> idxs;
  while (infile >> idx)
  {
    idxs.push_back(idx);
  }
  for (const auto& node : topological_map_->graph_vertexes_)
  {
    int idx = topological_map_->map_[node].index_;
    if (std::find(idxs.begin(), idxs.end(), idx) != idxs.end())
    {
      topological_map_->map_[node].has_file_ = true;
    }
  }

  // Parse the planes (since we only need to read them once)
  Parameters l_params;
  l_params.map_input_file_ = params_.grid_map_files_folder_ + "planes.xml";
  OccupancyMap tmp_grid_map(l_params, Pose(0, 0, 0, 0, 0, 0), 0, 0);
  MapParser mp(l_params);
  mp.parseHeader(&l_params);
  mp.parseFile(&tmp_grid_map);
  topological_map_->planes_ = tmp_grid_map.planes_;
}

void LocalizationNode::getTfs()
{
  tf2_ros::Buffer tf_buffer(this->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Base to laser tf
  geometry_msgs::msg::TransformStamped base2laser_msg;
  bool got_base2laser = false;
  while (!got_base2laser && rclcpp::ok())
  {
    try
    {
      base2laser_msg = tf_buffer.lookupTransform(params_.lidar_sensor_frame_, params_.base_frame_id_, rclcpp::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      continue;
    }
    got_base2laser = true;
  }

  // Base to IMU tf
  geometry_msgs::msg::TransformStamped base2imu_msg;
  bool got_base2imu = false;
  while (!got_base2imu && (params_.use_imu_ || params_.use_gyroscope_) && rclcpp::ok())
  {
    try
    {
      base2imu_msg = tf_buffer.lookupTransform(params_.imu_sensor_frame_, params_.base_frame_id_, rclcpp::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
      continue;
    }
    got_base2imu = true;
  }

  // Save base to laser tf
  tf2::Stamped<tf2::Transform> base2laser_stamped;
  tf2::fromMsg(base2laser_msg, base2laser_stamped);
  tf2::Transform base2laser = base2laser_stamped;
  tf2::Vector3 t = base2laser.getOrigin();
  tf2Scalar roll, pitch, yaw;
  base2laser.getBasis().getRPY(roll, pitch, yaw);
  lid_mapper_->setLaser2Base(t.getX(), t.getY(), t.getZ(), roll, pitch, yaw);

  // Save base to imu tf
  tf2::fromMsg(base2imu_msg.transform.rotation, input_data_.base2imu_quat_);

  // Initialize tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void LocalizationNode::getTfsFromParameters()
{
  std::string prefix = this->get_name();
  std::string param;

  std::vector<double> base_to_lidar(6);

  param = prefix + ".base_to_lidar_tf";
  this->declare_parameter(param);
  if (!this->get_parameter(param, base_to_lidar))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }

  lid_mapper_->setLaser2Base(base_to_lidar[0], base_to_lidar[1], base_to_lidar[2], base_to_lidar[3], base_to_lidar[4], base_to_lidar[5]);

  // Initialize tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void LocalizationNode::loop()
{
  // Reset information flags
  input_data_.received_scans_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

  // ---------------------------------------------------------
  // ----- Initialize robot's location
  // ---------------------------------------------------------
  publishTopologicalMap();
  initializeLocalization();

  while (rclcpp::ok())
  {
    loopOnce();
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }
}

void LocalizationNode::loopOnce()
{
  // Check if we have all the necessary data
  bool can_continue = input_data_.received_scans_;

  if (!can_continue)
  {
    return;
  }

  // VineSLAM main loop
  Timer l_timer("vineslam_ros");
  if (!init_flag_ && !init_odom_)
  {
    l_timer.tick("vineslam_ros::process()");
    process();
    l_timer.tock();
  }

  // Broadcast tfs
#if VERBOSE == 1
  timer_->tick("vineslam_ros::broadcastTfs()");
#endif
  broadcastTfsWithTopic();
#if VERBOSE == 1
  timer_->tock();
#endif

  // Publish VineSLAM state
  std_msgs::msg::String vineslam_state;
  if (init_flag_)
  {
    vineslam_state.data = "VINESLAM_WAITING_FOR_INITIALIZATION";
  }
  else
  {
    if (localization_error_counter_ >= 10)  // we wait for vineslam to fail for 10 straight times before we enter in error mode
    {
      vineslam_state.data = "VINESLAM_ERROR";
    }
    else
    {
      vineslam_state.data = "VINESLAM_OK";
    }
  }
  vineslam_state_publisher_->publish(vineslam_state);

  // Reset information flags
  input_data_.received_scans_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

#if VERBOSE == 1
  RCLCPP_INFO(this->get_logger(), "\n-------\n%s%s-------\n", timer_->getLog().c_str(), l_timer.getLog().c_str());
  timer_->clearLog();
#endif
}

void LocalizationNode::init()
{
  // ---------------------------------------------------------
  // ----- Initialize the localizer and get first particles distribution
  // ---------------------------------------------------------
  localizer_->init(robot_pose_, localization_mapping_interface_);
  localizer_->changeGPSFlag(params_.use_gps_ && params_.use_gps_heading_);
  robot_pose_ = localizer_->getPose();

  // ---------------------------------------------------------
  // ----- Initialize the multi-layer maps
  // ---------------------------------------------------------

  // - 3D PCL corner map estimation
  std::vector<Corner> l_corners;
  std::vector<Planar> l_planars;
  std::vector<SemiPlane> l_planes;
  SemiPlane l_ground_plane;
  if (params_.use_lidar_features_)
  {
#if LIDAR_TYPE == 0 || LIDAR_TYPE == 1 || LIDAR_TYPE == 3
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
#elif LIDAR_TYPE == 2 || LIDAR_TYPE == 4
    true;
    // Here we don't do anything because livox's feature extractor does not like to extract features when the system
    // is initializing
    // For LIDAR_TYPE = 4 we don't extract features, we use all points
#endif
  }
}

void LocalizationNode::process()
{
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------
  // ---- Localization and mapping procedures
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------

  // ---------------------------------------------------------
  // ----- Prepare topological map
  // ---------------------------------------------------------
  topological_map_->getActiveNodes(robot_pose_);
  topological_map_->allocateNodes(robot_pose_, input_data_.scan_pts_);

  // ---------------------------------------------------------
  // ----- Build local maps to use in the localization
  // ---------------------------------------------------------

  // - Compute 3D PCL corners and ground plane on robot's referential frame
  std::vector<Corner> l_corners;
  std::vector<Planar> l_planars;
  std::vector<SemiPlane> l_planes;
  SemiPlane l_ground_plane;
  if (params_.use_lidar_features_)
  {
#if VERBOSE == 1
    timer_->tick("lidar_mapper::localMap()");
#endif
#if LIDAR_TYPE == 0 || LIDAR_TYPE == 1 || LIDAR_TYPE == 3
    lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
#elif LIDAR_TYPE == 2
    lid_mapper_->localMap(input_data_.scan_pts_, header_.stamp.sec, l_corners, l_planars, l_planes, l_ground_plane);
#elif LIDAR_TYPE == 4
    // For LIDAR_TYPE = 4 we don't extract features, we use all points
    for (size_t i = 0; i < input_data_.scan_pts_.size(); i++)
    {
      Planar ft(input_data_.scan_pts_[i], 0);
      l_planars.push_back(ft);
    }
#endif
#if VERBOSE == 1
    timer_->tock();
#endif
  }

  // ---------------------------------------------------------
  // ----- Build observation structure to use in the localization
  // ---------------------------------------------------------
  // * High level landmarks (if we're using them)
  // * Point cloud corners and planars
  // * GPS (if we're using it)
  obsv_.corners_ = l_corners;
  obsv_.planars_ = l_planars;
  obsv_.ground_plane_ = l_ground_plane;
  obsv_.planes_ = l_planes;
  obsv_.gps_pose_ = input_data_.gnss_pose_;
  obsv_.gps_heading_ = input_data_.gnss_heading_;
  obsv_.imu_pose_ = input_data_.imu_pose_;

  // ---------------------------------------------------------
  // ----- Localization procedure
  // ---------------------------------------------------------
  Tf p_odom_tf = input_data_.p_wheel_odom_pose_.toTf();
  Tf c_odom_tf = input_data_.wheel_odom_pose_.toTf();
  Tf odom_inc_tf = p_odom_tf.inverse() * c_odom_tf;
  Pose odom_inc(odom_inc_tf.R_array_, odom_inc_tf.t_array_);
  input_data_.p_wheel_odom_pose_ = input_data_.wheel_odom_pose_;
  odom_inc.normalize();

  // Fuse odometry and gyroscope to get the innovation pose
  Pose innovation;
  if (params_.use_gyroscope_)
  {
    double yaw_inc = Const::normalizeAngle(input_data_.imu_data_pose_.Y_ - input_data_.p_imu_data_pose_.Y_);
    Pose imu_inc(0.0, 0.0, 0.0, 0.0, 0.0, yaw_inc);
    input_data_.p_imu_data_pose_ = input_data_.imu_data_pose_;
    computeInnovation(odom_inc, imu_inc, innovation);
  }
  else
  {
    innovation = odom_inc;
  }

  localizer_->process(innovation, obsv_, topological_map_);
  robot_pose_ = localizer_->getPose();
#if VERBOSE == 1
  timer_->tick("localizer::getPrecision()");
#endif
  uint32_t precision = localizer_->getPrecision(l_planars, topological_map_);
  if (precision < min_precision_value_)
  {
    localization_error_counter_++;
  }
  else
  {
    localization_error_counter_ = 0;
  }
#if VERBOSE == 1
  timer_->tock();
#endif

  // ---------------------------------------------------------
  // ----- ROS publishers
  // ---------------------------------------------------------
  tf2::Quaternion q;
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  q.normalize();

  // Convert vineslam pose to ROS pose and publish it
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = header_.stamp;
  pose_stamped.header.frame_id = params_.world_frame_id_;
  pose_stamped.pose.position.x = robot_pose_.x_;
  pose_stamped.pose.position.y = robot_pose_.y_;
  pose_stamped.pose.position.z = robot_pose_.z_;
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();
  pose_publisher_->publish(pose_stamped);

  // Convert set of particles into a ROS pose array and publish it
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = header_.stamp;
  pose_array.header.frame_id = params_.world_frame_id_;
  for (const auto& particle : localizer_->pf_->particles_)
  {
    tf2::Quaternion q_particle;
    q_particle.setRPY(particle.p_.R_, particle.p_.P_, particle.p_.Y_);
    q_particle.normalize();

    pose.position.x = particle.p_.x_;
    pose.position.y = particle.p_.y_;
    pose.position.z = particle.p_.z_;
    pose.orientation.x = q_particle.x();
    pose.orientation.y = q_particle.y();
    pose.orientation.z = q_particle.z();
    pose.orientation.w = q_particle.w();
    pose_array.poses.push_back(pose);
  }
  poses_publisher_->publish(pose_array);

  // Publish maps
#if VERBOSE == 1
  timer_->tick("publish3DMap()");
#endif
  if (params_.publish_level_ == 1)
  {
    publish3DMap(l_planars, planars_local_publisher_);
  }
  else if (params_.publish_level_ == 2)
  {
    publish3DMap(l_planars, planars_local_publisher_);

    sensor_msgs::msg::PointCloud2 planar_cloud2;
    Convertions::toROSMsg(topological_map_->getPlanars(), planar_cloud2);
    planar_cloud2.header.frame_id = params_.world_frame_id_;
    map3D_planars_publisher_->publish(planar_cloud2);
  }
#if VERBOSE == 1
  timer_->tock();
#endif

  // ---------------------------------------------------------
  // ----- Check and update topological map using the new robot pose
  // ---------------------------------------------------------
#if VERBOSE == 1
  timer_->tick("topological_map::deallocateNodes()");
#endif
  std::vector<vertex_t> nodes_to_delete;
  topological_map_->deactivateNodes(robot_pose_, nodes_to_delete);
  std::thread deallocate_nodes_th(&TopologicalMap::deallocateNodes, topological_map_, nodes_to_delete, false);
  deallocate_nodes_th.detach();
#if VERBOSE == 1
  timer_->tock();
#endif
}

void LocalizationNode::broadcastTfsWithTopic()
{
  tf2::Quaternion q;

  // Get base to map transformation
  geometry_msgs::msg::TransformStamped base2map_msg;
  base2map_msg.header.stamp = header_.stamp;
  base2map_msg.header.frame_id = params_.world_frame_id_;
  base2map_msg.child_frame_id = params_.base_frame_id_;
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  q.normalize();
  Convertions::pose2TransformStamped(q, tf2::Vector3(robot_pose_.x_, robot_pose_.y_, robot_pose_.z_), base2map_msg);

  // Get odom to base transformation
  geometry_msgs::msg::TransformStamped odom2base_msg;
  base2map_msg.header.stamp = header_.stamp;
  base2map_msg.header.frame_id = params_.odom_frame_id_;
  base2map_msg.child_frame_id = params_.base_frame_id_;
  q = tf2::Quaternion(latest_odom_msg_.pose.pose.orientation.x, latest_odom_msg_.pose.pose.orientation.y, latest_odom_msg_.pose.pose.orientation.z, latest_odom_msg_.pose.pose.orientation.w);
  q.normalize();
  Convertions::pose2TransformStamped(q, tf2::Vector3(latest_odom_msg_.pose.pose.position.x, latest_odom_msg_.pose.pose.position.y, latest_odom_msg_.pose.pose.position.z), odom2base_msg);

  // Compute odom to map transformation
  tf2::Stamped<tf2::Transform> odom2base_tf, base2map_tf, odom2map_tf;
  tf2::fromMsg(odom2base_msg, odom2base_tf);
  tf2::fromMsg(base2map_msg, base2map_tf);

  geometry_msgs::msg::TransformStamped odom2map_msg;

  tf2::Transform t = odom2base_tf * base2map_tf.inverse();
  odom2map_tf.setRotation(t.getRotation());
  odom2map_tf.setOrigin(t.getOrigin());
  Convertions::pose2TransformStamped(odom2map_tf.getRotation(), odom2map_tf.getOrigin(), odom2map_msg);

  odom2map_msg.header.stamp = header_.stamp;
  odom2map_msg.header.frame_id = params_.odom_frame_id_;
  odom2map_msg.child_frame_id = params_.world_frame_id_;

  // Broadcast transformation
  tf_broadcaster_->sendTransform(odom2map_msg);
}

void LocalizationNode::broadcastTfs()
{
  tf2::Quaternion q;

  // Get base to map transformation
  geometry_msgs::msg::TransformStamped base2map_msg;
  base2map_msg.header.stamp = header_.stamp;
  base2map_msg.header.frame_id = params_.world_frame_id_;
  base2map_msg.child_frame_id = params_.base_frame_id_;
  q.setRPY(robot_pose_.R_, robot_pose_.P_, robot_pose_.Y_);
  q.normalize();
  Convertions::pose2TransformStamped(q, tf2::Vector3(robot_pose_.x_, robot_pose_.y_, robot_pose_.z_), base2map_msg);

  geometry_msgs::msg::TransformStamped odom2base_msg;
  tf2_ros::Buffer tf_buffer(this->get_clock());
  tf2_ros::TransformListener tfListener(tf_buffer);

  try
  {
    // Get odom to base transformation
    odom2base_msg = tf_buffer.lookupTransform(params_.odom_frame_id_, params_.base_frame_id_, rclcpp::Time(0), rclcpp::Duration(300000000));

    tf2::Stamped<tf2::Transform> odom2base_tf, base2map_tf, odom2map_tf;
    tf2::fromMsg(odom2base_msg, odom2base_tf);
    tf2::fromMsg(base2map_msg, base2map_tf);

    geometry_msgs::msg::TransformStamped odom2map_msg;

    // Compute odom to map transformation
    tf2::Transform t = odom2base_tf * base2map_tf.inverse();
    odom2map_tf.setRotation(t.getRotation());
    odom2map_tf.setOrigin(t.getOrigin());
    Convertions::pose2TransformStamped(odom2map_tf.getRotation(), odom2map_tf.getOrigin(), odom2map_msg);

    odom2map_msg.header.stamp = header_.stamp;
    odom2map_msg.header.frame_id = params_.odom_frame_id_;
    odom2map_msg.child_frame_id = params_.world_frame_id_;

    // Broadcast transformation
    tf_broadcaster_->sendTransform(odom2map_msg);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
  }
}

void LocalizationNode::setupInitializationMarker()
{
  // -------------------------------------------------------------------------------
  // ------ Compute the difference between map's and robot's datums
  // -------------------------------------------------------------------------------
  double robot_e, robot_n, robot_u;
  if (params_.use_gps_ && params_.use_gps_heading_)
  {
    while (init_gps_ || init_gps_heading_)  // wait until we get gnns and heading
    {
      RCLCPP_WARN(this->get_logger(), "\033[1;33mWaiting for GNSS to initialize interactive marker...\033[0m");
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    geodetic_converter_->geodetic2enu(input_data_.first_gnss_pose_.latitude, input_data_.first_gnss_pose_.longitude, params_.map_datum_alt_, robot_e, robot_n, robot_u);
    robot_pose_ = Pose(robot_e, robot_n, robot_u, 0, 0, input_data_.gnss_heading_);
  }
  else
  {
    robot_pose_ = Pose(0, 0, 0, 0, 0, 0);
  }

  // Create the interactive marker menu entries
  im_menu_handler_ = interactive_markers::MenuHandler();
  im_menu_handler_.insert("Set pose", std::bind(&LocalizationNode::iMenuCallback, this, std::placeholders::_1));
  im_menu_handler_.insert("Republish global maps", std::bind(&LocalizationNode::iMenuCallback, this, std::placeholders::_1));

  // Declare the interactive marker server
  im_server_ =
      std::make_unique<interactive_markers::InteractiveMarkerServer>("vineslam/initialization_marker", get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());

  // Create the 6-DoF interactive marker
  std::string marker_name = "initialization_markers";
  visualization_msgs::msg::InteractiveMarker imarker;
  make6DofMarker(imarker, robot_pose_, marker_name);

  // Insert the interactive marker and menu into the server and associate them with the corresponding callback functions
  im_server_->insert(imarker);
  im_server_->setCallback(imarker.name, std::bind(&LocalizationNode::iMarkerCallback, this, std::placeholders::_1));
  im_menu_handler_.apply(*im_server_, imarker.name);

  // 'Commit' changes and send to all clients
  im_server_->applyChanges();
}

void LocalizationNode::iMarkerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  if (init_flag_)
  {
    // Save the robot initial pose set by the user
    tf2::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
    tf2Scalar R, P, Y;
    tf2::Matrix3x3(q).getRPY(R, P, Y);

    robot_pose_.x_ = feedback->pose.position.x;
    robot_pose_.y_ = feedback->pose.position.y;
    robot_pose_.z_ = feedback->pose.position.z;
    robot_pose_.R_ = static_cast<float>(R);
    robot_pose_.P_ = static_cast<float>(P);
    robot_pose_.Y_ = static_cast<float>(Y);

    // Since the robot pose changed, we have to change the allocated nodes on the topological map
    topological_map_->getActiveNodes(robot_pose_);
    topological_map_->allocateNodes(robot_pose_, input_data_.scan_pts_);

    // And deallocate the ones that are not needed anymore
    std::vector<vertex_t> nodes_to_delete;
    topological_map_->deactivateNodes(robot_pose_, nodes_to_delete);
    std::thread deallocate_nodes_th(&TopologicalMap::deallocateNodes, topological_map_, nodes_to_delete, false);
    deallocate_nodes_th.detach();

    // Then we republish them
    publishTopologicalMap();

    sensor_msgs::msg::PointCloud2 planar_cloud2;
    Convertions::toROSMsg(topological_map_->getPlanars(), planar_cloud2);
    planar_cloud2.header.frame_id = params_.world_frame_id_;
    map3D_planars_publisher_->publish(planar_cloud2);

    // Publish local maps to match with global map for initialization
    // - Compute 3D PCL corners and ground plane on robot's referential frame
    std::vector<Corner> l_corners;
    std::vector<Planar> l_planars;
    std::vector<SemiPlane> l_planes;
    SemiPlane l_ground_plane;
    if (params_.use_lidar_features_)
    {
#if VERBOSE == 1
      timer_->tick("lidar_mapper::localMap()");
#endif
#if LIDAR_TYPE == 0 || LIDAR_TYPE == 1 || LIDAR_TYPE == 3
      lid_mapper_->localMap(input_data_.scan_pts_, l_corners, l_planars, l_planes, l_ground_plane);
#elif LIDAR_TYPE == 2
      lid_mapper_->localMap(input_data_.scan_pts_, header_.stamp.sec, l_corners, l_planars, l_planes, l_ground_plane);
#elif LIDAR_TYPE == 4
    // For LIDAR_TYPE = 4 we don't extract features, we use all points
    for (size_t i = 0; i < input_data_.scan_pts_.size(); i++)
    {
      Planar ft(input_data_.scan_pts_[i], 0);
      l_planars.push_back(ft);
    }
#endif
#if VERBOSE == 1
      timer_->tock();
#endif
      publish3DMap(l_planars, planars_local_publisher_);
    }
  }
}

void LocalizationNode::iMenuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  if (feedback->menu_entry_id == 1)
  {
    // Initialize the localization system
    std::cout << "\033[1;33mInitializing system...\033[0m\n" << std::flush;
    init();
    init_flag_ = false;
    std::cout << "\033[1;32mSystem running (!)\033[0m\n" << std::flush;
  }
  else if (feedback->menu_entry_id == 2)
  {
    std::cout << "\033[1;34mRepublishing global maps...\033[0m\n" << std::flush;
    publishTopologicalMap();
    publish3DMap(topological_map_->getPlanars(), map3D_planars_publisher_);  // TODO (André Aguiar): this is wrong :p
  }
}

void LocalizationNode::initializeLocalization()
{
  if (params_.initialization_type_ == 0)  // interactive marker
  {
    init_flag_ = true;
    setupInitializationMarker();
  }
  else if (params_.initialization_type_ == 1)  // vineslam last pose
  {
    init_flag_ = true;
    std::string path = params_.topological_map_folder_ + "/vineslam_last_pose.txt";
    RCLCPP_INFO(this->get_logger(), "Path: %s", path.c_str());
    initializeFromFile(path);
  }
  else if (params_.initialization_type_ == 2)  // fixed pose
  {
    init_flag_ = true;
    std::string path = params_.topological_map_folder_ + "/vineslam_fixed_pose.txt";
    RCLCPP_INFO(this->get_logger(), "Path: %s", path.c_str());
    initializeFromFile(path);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Initialization type choosen is wrong. Valid values are 0, 1, or 2. Please fix this and reinitialize vineslam");
  }
}

void LocalizationNode::initializeFromFile(const std::string& path)
{
  std::ifstream file(path);
  if (file.is_open())
  {
    float n;
    int i = 0;
    char c;
    while (file >> n >> c && c == ',')
    {
      if (i == 0)
      {
        robot_pose_.x_ = n;
      }
      else if (i == 1)
      {
        robot_pose_.y_ = n;
      }
      else if (i == 2)
      {
        robot_pose_.z_ = n;
      }
      else if (i == 3)
      {
        robot_pose_.R_ = n;
      }
      else if (i == 4)
      {
        robot_pose_.P_ = n;
      }
      else if (i == 5)
      {
        robot_pose_.Y_ = n;
      }
      i++;
    }
    file.close();
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Initialization file not found");
  }

  std::cout << "\033[1;33mInitializing system...\033[0m\n" << std::flush;
  init();
  init_flag_ = false;
  std::cout << "\033[1;32mSystem running (!)\033[0m\n" << std::flush;
}

void LocalizationNode::initializeLocalizationSrv(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{
  initializeLocalization();
}

LocalizationNode::~LocalizationNode()
{
  std::ofstream file;
  std::string path = params_.topological_map_folder_ + "/vineslam_last_pose.txt";
  file.open(path);
  file << robot_pose_.x_ << "," << robot_pose_.y_ << "," << robot_pose_.z_ << "," << robot_pose_.R_ << "," << robot_pose_.P_ << "," << robot_pose_.Y_ << ",\n";
  file.close();
}

}  // namespace vineslam
