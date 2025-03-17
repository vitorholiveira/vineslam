#include "../include/slam_node.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vineslam::SLAMNode>());
  rclcpp::shutdown();

  return 0;
}

namespace vineslam
{
// --------------------------------------------------------------------------------
// ----- Constructor and destructor
// --------------------------------------------------------------------------------

SLAMNode::SLAMNode() : VineSLAM_ros("slam_node")
{
  // Load params
  loadParameters(params_);

  // Set initialization flags default values
  init_flag_ = true;
  init_gps_ = true;
  init_odom_ = true;
  init_gyro_ = true;

  // Initialize variables
  estimate_heading_ = params_.use_gps_;

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

  // ROS services
  save_map_srv_ = this->create_service<vineslam_ros::srv::SaveMap>("vineslam/save_map", std::bind(&VineSLAM_ros::saveMap, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1, std::placeholders::_2));

  // Semantic feature subscription
  landmark_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("detections_topic", 10, std::bind(&VineSLAM_ros::landmarkListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // Scan subscription
  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("scan_topic", 10, std::bind(&VineSLAM_ros::scanListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // Odometry subscription
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom_topic", 10, std::bind(&VineSLAM_ros::odomListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // GPS subscription
  gps_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps_topic", 10, std::bind(&VineSLAM_ros::gpsListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  // GPS heading subscription
  gps_heading_subscriber_ = this->create_subscription<ublox_msgs::msg::NavRELPOSNED9>("gps_heading_topic", 10, std::bind(&SLAMNode::gpsArduSimpleRoverCallBack, this, std::placeholders::_1));
  // IMU subscriptions
  imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("imu_topic", 10, std::bind(&VineSLAM_ros::imuListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));
  imu_data_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_data_topic", 10, std::bind(&VineSLAM_ros::imuDataListener, dynamic_cast<VineSLAM_ros*>(this), std::placeholders::_1));

  // Feature and map publishers
  topological_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vineslam/topological_map", 10);
  elevation_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vineslam/elevation_map", 10);
  map3D_corners_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vineslam/map3D/corners_map", 10);
  map3D_planars_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vineslam/map3D/planars_map", 10);
  map3D_planes_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vineslam/map3D/planes_map", 10);
  planes_local_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vineslam/map3D/planes_local", 10);
  corners_local_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vineslam/map3D/corners_local", 10);
  planars_local_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("vineslam/map3D/planars_local", 10);
  // Pose publishers
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/pose", 10);
  path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("vineslam/path", 10);
  poses_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("vineslam/poses", 10);
  // Input data publishers
  gps_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_gps_pose", 10);
  gps_heading_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_gps_heading", 10);
  mag_orientation_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_mag_orientation", 10);
  gyro_orientation_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("vineslam/input_gyro_orientation", 10);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("vineslam/input_odometry", 10);
  // Debug publishers
  grid_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vineslam/debug/grid_map_limits", 10);
  robot_box_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("vineslam/debug/robot_box", 10);

  // Allocate map memory
  RCLCPP_INFO(this->get_logger(), "\033[1;33mAllocating map memory...\033[0m");
  loadMaps();

  // Static transforms
  RCLCPP_INFO(this->get_logger(), "\033[1;33mWaiting for static transforms...\033[0m");
  getTfs();

  // Call execution threads
  std::thread th1(&SLAMNode::loop, this);
  th1.detach();
}

void SLAMNode::loadParameters(Parameters& params)
{
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
  param = "multilayer_mapping.topological_map.autogen_topological_map";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.autogen_topological_map_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.topological_map.folder";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.topological_map_folder_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.topological_map.dimensions.x";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.topological_map_dim_x_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.topological_map.dimensions.y";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.topological_map_dim_y_))
  {
    RCLCPP_WARN(this->get_logger(), "%s not found.", param.c_str());
  }
  param = "multilayer_mapping.topological_map.dimensions.square_size";
  this->declare_parameter(param);
  if (!this->get_parameter(param, params.topological_map_dim_square_size_))
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

void SLAMNode::loadMaps()
{
  elevation_map_ = new ElevationMap(params_, Pose(0, 0, 0, 0, 0, 0));

  // Load topological map
  topological_map_ = new TopologicalMap(params_);
  if (params_.autogen_topological_map_)
  {
    genTopologicalMap();
    topological_map_->active_nodes_ref_.resize(topological_map_->graph_vertexes_.size());
  }
  else
  {
    TopologicalMapParser topological_map_parser(params_);
    topological_map_parser.parseFile(topological_map_);
    topological_map_->active_nodes_ref_.resize(topological_map_->graph_vertexes_.size());
  }

  // Allocate memory for the localization and mapping interface structure
  localization_mapping_interface_ = new LocalizationMappingInterface(topological_map_->graph_vertexes_.size());
}

void SLAMNode::getTfs()
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

void SLAMNode::loop()
{
  // Reset information flags
  input_data_.received_scans_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

  while (rclcpp::ok())
  {
    loopOnce();
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }
}

void SLAMNode::loopOnce()
{
  if (init_flag_)
  {
    RCLCPP_INFO(this->get_logger(), "\033[1;33mInitializing system...\033[0m");
    // Call init function to initialize VineSLAM
    init();
    // ... is it initialized?
    if (!init_flag_)
    {
      RCLCPP_INFO(this->get_logger(), "\033[1;32mSystem running (!)\033[0m");
    }
  }

  // Check if we have all the necessary data
  bool can_continue = input_data_.received_scans_ && !init_odom_ && !init_flag_;

  if (!can_continue)
  {
    return;
  }

  // VineSLAM main loop
  Timer l_timer("vineslam_ros");
  l_timer.tick("vineslam_ros::process()");
  process();
  l_timer.tock();

  // Broadcast tfs
#if VERBOSE == 1
  timer_->tick("vineslam_ros::broadcastTfs()");
#endif
  broadcastTfsWithTopic();
#if VERBOSE == 1
  timer_->tock();
#endif

  // Publish global maps
#if VERBOSE == 1
  timer_->tick("vineslam_ros::publishDenseInfo()");
#endif
  publishDenseInfo();
#if VERBOSE == 1
  timer_->tock();
#endif

  // Reset information flags
  input_data_.received_scans_ = false;
  input_data_.received_odometry_ = false;
  input_data_.received_gnss_ = false;

#if VERBOSE == 1
  RCLCPP_INFO(this->get_logger(), "\n-------\n%s%s-------\n", timer_->getLog().c_str(), l_timer.getLog().c_str());
  timer_->clearLog();
#endif
}

void SLAMNode::init()
{
  // ---------------------------------------------------------
  // ----- Get robot's origin
  // ---------------------------------------------------------
  Pose map_origin(0, 0, 0, 0, 0, 0);
  bool use_gps = false;
  if (params_.use_gps_)
  {
    if (params_.use_gps_heading_)
    {
      if (!init_gps_ && !init_gps_heading_)
      {
        // Calculate robot's origin
        double n, e, d;
        geodetic_converter_->geodetic2ned(input_data_.first_gnss_pose_.latitude, input_data_.first_gnss_pose_.longitude, input_data_.first_gnss_pose_.altitude, n, e, d);
        map_origin.x_ = e;
        map_origin.y_ = n;
        map_origin.z_ = d;
        map_origin.R_ = 0;
        map_origin.P_ = 0;
        map_origin.Y_ = input_data_.gnss_heading_;  // robot's first heading is the one that we have from the GNSS heading sensor since we are working on ENU

        use_gps = true;  // in this case we have GNSS and heading, so we can use them for localization
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "\033[1;33mWaiting for GNSS or heading data...\033[0m");
        return;
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "\033[1;33mTrying to use GNSS without using GNSS heading. This is not supported at the moment! Initilizing without GNSS.\033[0m");
    }
  }
  else
  {
    // In this case we do not do anything
    // Since we do not have GNSS, our datum is (0, 0, 0, 0, 0, 0)
    true;  // :)
  }

  // ---------------------------------------------------------
  // ----- Initialize the localizer and get first particles distribution
  // ---------------------------------------------------------
  localizer_->init(map_origin, localization_mapping_interface_);
  localizer_->changeGPSFlag(use_gps);
  robot_pose_ = localizer_->getPose();

  // ---------------------------------------------------------
  // ----- Prepare the topological map
  // ---------------------------------------------------------
  topological_map_->polar2Enu(geodetic_converter_);
  // We do this init function on a thread because it is time expensive, and we can carry on without it
  topological_map_->init_function_called_ = true;
  std::thread th(&TopologicalMap::init, topological_map_, 0.0, localization_mapping_interface_, false);
  th.detach();

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

  // - Register 3D maps
  if (topological_map_->is_initialized_)
  {
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *topological_map_);
    topological_map_->downsamplePlanars();
  }
  else
  {
    lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *topological_map_->tmp_grid_map_, *elevation_map_);
    topological_map_->tmp_grid_map_->downsamplePlanars();
  }

  // VineSLAM has been initted
  init_flag_ = false;
}

void SLAMNode::process()
{
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------
  // ---- Localization and mapping procedures
  // -------------------------------------------------------------------------------
  // -------------------------------------------------------------------------------

#if VERBOSE == 1
  timer_->tick("topological_map::getActiveNodes()");
#endif
  topological_map_->getActiveNodes(robot_pose_);
#if VERBOSE == 1
  timer_->tock();
#endif

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

  if (topological_map_->is_initialized_ || topological_map_->tmp_grid_map_ == nullptr)
  {
    localizer_->process(innovation, obsv_, topological_map_);
  }
  else
  {
    localizer_->process(innovation, obsv_, topological_map_->tmp_grid_map_);
  }
  robot_pose_ = localizer_->getPose();

  // ---------------------------------------------------------
  // ----- Register multi-layer map
  // ---------------------------------------------------------
  if (params_.use_lidar_features_)
  {
    if (topological_map_->is_initialized_)
    {
      lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *topological_map_);
      topological_map_->downsamplePlanars();
    }
    else
    {
      lid_mapper_->registerMaps(robot_pose_, l_corners, l_planars, l_planes, l_ground_plane, *topological_map_->tmp_grid_map_);
      topological_map_->tmp_grid_map_->downsamplePlanars();
    }
  }
  // ---------------------------------------------------------
  // ----- Check and update topological map using the new robot pose
  // ---------------------------------------------------------
#if VERBOSE == 1
  timer_->tick("topological_map::deallocateNodes()");
#endif
  std::vector<vertex_t> nodes_to_delete;
  topological_map_->deactivateNodes(robot_pose_, nodes_to_delete);
  std::thread deallocate_nodes_th(&TopologicalMap::deallocateNodes, topological_map_, nodes_to_delete, true);
  deallocate_nodes_th.detach();
#if VERBOSE == 1
  timer_->tock();

  timer_->tick("vineslam_ros::local_publishers()");
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

  // Push back the current pose to the path container and publish it
  path_.push_back(pose_stamped);
  nav_msgs::msg::Path ros_path;
  ros_path.header.stamp = rclcpp::Time();
  ros_path.header.frame_id = params_.world_frame_id_;
  ros_path.poses = path_;
  path_publisher_->publish(ros_path);

  // Non-dense map publishers
  publish3DMap(l_corners, corners_local_publisher_);
  publish3DMap(l_planars, planars_local_publisher_);
  l_planes.push_back(l_ground_plane);
  publish3DMap(l_planes, planes_local_publisher_);
  publishRobotBox(robot_pose_);

#if VERBOSE == 1
  timer_->tock();
#endif
}

void SLAMNode::broadcastTfsWithTopic()
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

void SLAMNode::broadcastTfs()
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

void SLAMNode::genTopologicalMap()
{
  double orig_x, orig_y;
  std::string datum_utm_zone;
  Convertions::GNSS2UTM(params_.map_datum_lat_, params_.map_datum_long_, orig_x, orig_y, datum_utm_zone);

  std::vector<vineslam::Node> vertexes;
  int i = 0;
  for (float xx = -params_.topological_map_dim_x_ / 2.; xx < params_.topological_map_dim_x_ / 2.;)
  {
    for (float yy = -params_.topological_map_dim_y_ / 2.; yy < params_.topological_map_dim_y_ / 2.;)
    {
      // Find square vertexes
      float square_lat1, square_lon1;
      double x1 = orig_x + (xx + 0.);
      double y1 = orig_y - (yy + 0.);
      Convertions::UTMtoGNSS(x1, y1, datum_utm_zone, square_lat1, square_lon1);

      float square_lat2, square_lon2;
      double x2 = orig_x + (xx + 0.);
      double y2 = orig_y - (yy + params_.topological_map_dim_square_size_);
      Convertions::UTMtoGNSS(x2, y2, datum_utm_zone, square_lat2, square_lon2);

      float square_lat3, square_lon3;
      double x3 = orig_x + (xx + params_.topological_map_dim_square_size_);
      double y3 = orig_y - (yy + params_.topological_map_dim_square_size_);
      Convertions::UTMtoGNSS(x3, y3, datum_utm_zone, square_lat3, square_lon3);

      float square_lat4, square_lon4;
      double x4 = orig_x + (xx + params_.topological_map_dim_square_size_);
      double y4 = orig_y - (yy + 0.);
      Convertions::UTMtoGNSS(x4, y4, datum_utm_zone, square_lat4, square_lon4);

      float center_lat, center_lon;
      double x5 = orig_x + (xx + params_.topological_map_dim_square_size_ / 2.);
      double y5 = orig_y - (yy + params_.topological_map_dim_square_size_ / 2.);
      Convertions::UTMtoGNSS(x5, y5, datum_utm_zone, center_lat, center_lon);

      vineslam::Node v;
      v.aligned_rectangle_.resize(2);
      v.rectangle_.resize(4);
      v.index_ = i++;
      v.center_.x_ = x1 + params_.topological_map_dim_square_size_ / 2.;
      v.center_.y_ = y1 + params_.topological_map_dim_square_size_ / 2.;
      v.center_.lat_ = center_lat;
      v.center_.lon_ = center_lon;
      v.aligned_rectangle_[0].x_ = 0;
      v.aligned_rectangle_[0].y_ = 0;
      v.aligned_rectangle_[1].x_ = 0;
      v.aligned_rectangle_[1].y_ = 0;
      v.aligned_rectangle_[0].lat_ = 0;
      v.aligned_rectangle_[0].lon_ = 0;
      v.aligned_rectangle_[1].lat_ = 0;
      v.aligned_rectangle_[1].lon_ = 0;
      v.rectangle_[0].x_ = 0;
      v.rectangle_[0].y_ = 0;
      v.rectangle_[1].x_ = 0;
      v.rectangle_[1].y_ = 0;
      v.rectangle_[2].x_ = 0;
      v.rectangle_[2].y_ = 0;
      v.rectangle_[3].x_ = 0;
      v.rectangle_[3].y_ = 0;
      v.rectangle_[0].lat_ = square_lat1;
      v.rectangle_[0].lon_ = square_lon1;
      v.rectangle_[1].lat_ = square_lat2;
      v.rectangle_[1].lon_ = square_lon2;
      v.rectangle_[2].lat_ = square_lat3;
      v.rectangle_[2].lon_ = square_lon3;
      v.rectangle_[3].lat_ = square_lat4;
      v.rectangle_[3].lon_ = square_lon4;
      v.rectangle_orientation_ = 0.;

      vertexes.push_back(v);

      yy += params_.topological_map_dim_square_size_;
    }
    xx += params_.topological_map_dim_square_size_;
  }

  // Insert vertexes into the graph
  for (const auto& v : vertexes)
  {
    vertex_t u = boost::add_vertex(v, topological_map_->map_);
    topological_map_->graph_vertexes_.push_back(u);
  }

  // Create directories for map saving
  std::experimental::filesystem::create_directories(params_.grid_map_files_folder_);
  std::experimental::filesystem::create_directories(params_.map_output_folder_);

  // Write topological map to file
  TopologicalMapWriter topological_map_writer(params_);
  topological_map_writer.write(params_, topological_map_);
}

SLAMNode::~SLAMNode()
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

  // ----------------------------------------------------
  // ------ Export point cloud map to ply
  // ----------------------------------------------------
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& node : topological_map_->saved_nodes_)
  {
    topological_map_->map_[node].has_file_ = true;
    topological_map_->allocateNodeMapFromFile(node);
    topological_map_->active_nodes_vertexes_.push_back(node);
  }

  std::vector<vineslam::Planar> all_planars = topological_map_->getPlanars();
  for (const auto& p : all_planars)
  {
    pcl::PointXYZ pt;
    pt.x = p.pos_.x_;
    pt.y = p.pos_.y_;
    pt.z = p.pos_.z_;

    cloud->push_back(pt);
  }

  // Save current cloud to file
  std::string ply_path = params_.topological_map_folder_ + "/output_map/map_vineslam.ply";
  pcl::io::savePLYFile(ply_path, *cloud);
}

}  // namespace vineslam
