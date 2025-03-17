#include "../../include/vineslam/localization/pf.hpp"

namespace vineslam
{
PF::PF(const Parameters& params, const Pose& initial_pose) : params_(params)
{
  // - General parameters
  use_lidar_features_ = params.use_lidar_features_;
  use_vertical_planes_ = params.use_vertical_planes_;
  use_ground_plane_ = params.use_ground_plane_;
  use_gps_ = params.use_gps_;
  use_gps_altitude_ = params.use_gps_altitude_;
  use_gps_heading_ = params.use_gps_heading_;
  use_imu_ = params.use_imu_;
  particles_size_ = params.number_particles_;

  // Initialize thread pool
  thread_pool_ = new lama::ThreadPool;
  thread_pool_->init(NUM_THREADS);

  // Initialize profiler
  t_ = new Timer("Particle Filter");

  // Initialize normal distributions
  particles_.resize(particles_size_);

  // Initialize all particles
  for (size_t i = 0; i < particles_size_; i++)
  {
    // Calculate the initial pose for each particle considering
    // - the input initial pose
    // - a sample distribution to spread the particles
    Pose m_pose = initial_pose + Pose(sampleGaussian(0.2), sampleGaussian(0.2), sampleGaussian(0.2), sampleGaussian(1 * DEGREE_TO_RAD), sampleGaussian(1 * DEGREE_TO_RAD), sampleGaussian(3 * DEGREE_TO_RAD));
    // Compute initial weight of each particle
    float weight = 1.;
    // Insert the particle into the particles array
    particles_[i] = Particle(i, m_pose, weight);
  }

  // Set filter settings
  sigma_landmark_matching_ = 0.1;
  sigma_feature_matching_ = 0.1;
  sigma_corner_matching_ = 0.1;
  sigma_planar_matching_ = 0.1;
  sigma_plane_matching_vector_ = 0.02;
  sigma_plane_matching_centroid_ = 0.10;
  sigma_gps_ = 0.1;
  sigma_gps_heading_ = 3.0 * DEGREE_TO_RAD;
  sigma_imu_ = 3.0 * DEGREE_TO_RAD;
  simple_weight_factor_ = 20.0;

  // Set normalizers (if we are using the simple weight version for corners and planars, they will be overwritten)
  normalizer_plane_vector_ = static_cast<float>(1.) / (sigma_plane_matching_vector_ * std::sqrt(M_2PI));
  normalizer_plane_centroid_ = static_cast<float>(1.) / (sigma_plane_matching_centroid_ * std::sqrt(M_2PI));
  normalizer_planar_ = static_cast<float>(1.) / (sigma_planar_matching_ * std::sqrt(M_2PI));
  normalizer_corner_ = static_cast<float>(1.) / (sigma_corner_matching_ * std::sqrt(M_2PI));
  normalizer_imu_ = static_cast<float>(1.) / (sigma_imu_ * std::sqrt(M_2PI));
  normalizer_gps_ = static_cast<float>(1.) / (sigma_gps_ * std::sqrt(M_2PI));
  normalizer_gps_heading_ = static_cast<float>(1.) / (sigma_gps_heading_ * std::sqrt(M_2PI));
}

// Samples a zero mean Gaussian
// See https://www.taygeta.com/random/gaussian.html
float PF::sampleGaussian(const float& sigma, const unsigned long int& S)
{
  if (S != 0)
    srand48(S);
  if (sigma == 0)
    return 0.;

  float x1, x2, w;
  float r;

  do
  {
    do
    {
      r = drand48();
    } while (r == 0.0);
    x1 = 2.0 * r - 1.0;
    do
    {
      r = drand48();
    } while (r == 0.0);
    x2 = 2.0 * drand48() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w > 1.0 || w == 0.0);

  return (sigma * x2 * sqrt(-2.0 * log(w) / w));
}

void PF::motionModel(const Pose& odom_inc)
{
  float d_trans = odom_inc.norm3D();

  // Innovate particles
  for (auto& particle : particles_)
  {
    // Build pose noise transformation matrix
    Pose pose_noise;
    pose_noise.x_ = params_.sigma_xx_ * d_trans * sampleGaussian(1.0) + params_.sigma_YY_ * std::fabs(odom_inc.Y_) * sampleGaussian(1.0);  // xx
    pose_noise.y_ = params_.sigma_yy_ * d_trans * sampleGaussian(1.0) + params_.sigma_YY_ * std::fabs(odom_inc.Y_) * sampleGaussian(1.0);  // yy
    pose_noise.z_ = params_.sigma_zz_ * d_trans * sampleGaussian(1.0);                                                                     // zz
    pose_noise.R_ = params_.sigma_RR_ * d_trans * sampleGaussian(1.0);                                                                     // RR
    pose_noise.P_ = params_.sigma_PP_ * d_trans * sampleGaussian(1.0);                                                                     // PP
    pose_noise.Y_ = params_.sigma_YY_ * std::fabs(odom_inc.Y_) * sampleGaussian(1.0);  

    std::array<float, 9> R_noise{};
    pose_noise.toRotMatrix(R_noise);
    Tf odom_noise_tf(R_noise, std::array<float, 3>{ pose_noise.x_, pose_noise.y_, pose_noise.z_ });

    // Built odom increment transformation matrix
    std::array<float, 9> R_inc{};
    odom_inc.toRotMatrix(R_inc);
    Tf odom_inc_tf(R_inc, std::array<float, 3>{ odom_inc.x_, odom_inc.y_, odom_inc.z_ });

    // Final pose increment
    Tf innovation_tf = odom_inc_tf * odom_noise_tf;

    // Save info for the next iteration
    particle.pp_ = particle.p_;
    particle.ptf_ = particle.tf_;

    // Apply transformation
    particle.tf_ = particle.ptf_ * innovation_tf;
    particle.p_ = Pose(particle.tf_.R_array_, particle.tf_.t_array_);
  }
}

void PF::update(const std::vector<Corner>& corners, const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane, const Pose& gps_pose, const double& gps_heading, const Pose& imu_pose,
                OccupancyMap* grid_map)
{
  std::vector<SemiPlane> filtered_planes;
  if (use_vertical_planes_ || use_ground_plane_)
  {
    preprocessPlanes(prev_average_pose_, grid_map->planes_, filtered_planes);
  }

  for (auto& particle : particles_)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, corners, planars, planes, ground_plane, filtered_planes, gps_pose, gps_heading, imu_pose, grid_map, &particle]() {
#endif
      float corner_w = 0, planar_w = 0, ground_w = 0, planes_w = 0, gps_w = 0, gps_heading_w = 0, imu_w = 0;

      if (use_lidar_features_)
      {
        cornersFactorSimple(particle, corners, grid_map, corner_w);

        planarsFactorSimple(particle, planars, grid_map, planar_w);
      }

      if (use_vertical_planes_)
      {
        planesFactor(particle, planes, filtered_planes, planes_w);
      }

      if (use_ground_plane_)
      {
        planesFactor(particle, { ground_plane }, filtered_planes, ground_w);
      }

      if (use_gps_)
      {
        gpsFactor(particle, gps_pose, gps_w);
      }

      if (use_gps_heading_)
      {
        gpsHeadingFactor(particle, gps_heading, gps_heading_w);
      }

      if (use_imu_)
      {
        imuFactor(particle, imu_pose, imu_w);
      }

      float m_corner_w = (corner_w > 0.) ? corner_w : static_cast<float>(1.);
      float m_planar_w = (planar_w > 0.) ? planar_w : static_cast<float>(1.);
      float m_ground_w = (ground_w > 0.) ? ground_w : static_cast<float>(1.);
      float m_planes_w = (planes_w > 0.) ? planes_w : static_cast<float>(1.);
      float m_gps_w = (gps_w > 0.) ? gps_w : static_cast<float>(1.);
      float m_gps_heading_w = (gps_heading_w > 0.) ? gps_heading_w : static_cast<float>(1.);
      float m_imu_w = (imu_w > 0.) ? imu_w : static_cast<float>(1.);

      particle.w_ = m_corner_w * m_planar_w * m_ground_w * m_planes_w * m_gps_w * m_gps_heading_w * m_imu_w;

      w_sum_ += particle.w_;
#if NUM_THREADS > 1
    });
#endif
  }

#if NUM_THREADS > 1
  thread_pool_->wait();
#endif
}

void PF::update(const std::vector<Corner>& corners, const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane, const Pose& gps_pose, const double& gps_heading, const Pose& imu_pose,
                TopologicalMap* topological_map)
{
  std::vector<SemiPlane> filtered_planes;
  if (use_vertical_planes_ || use_ground_plane_)
  {
    preprocessPlanes(prev_average_pose_, topological_map->planes_, filtered_planes);
  }

  for (auto& particle : particles_)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, corners, planars, planes, ground_plane, filtered_planes, gps_pose, gps_heading, imu_pose, topological_map, &particle]() {
#endif
      float corner_w = 0, planar_w = 0, ground_w = 0, planes_w = 0, gps_w = 0, gps_heading_w = 0, imu_w = 0;

      if (use_lidar_features_)
      {
        cornersFactorSimple(particle, corners, topological_map, corner_w);

        planarsFactorSimple(particle, planars, topological_map, planar_w);
      }

      if (use_vertical_planes_)
      {
        planesFactor(particle, planes, filtered_planes, planes_w);
      }

      if (use_ground_plane_)
      {
        planesFactor(particle, { ground_plane }, filtered_planes, ground_w);
      }

      if (use_gps_)
      {
        gpsFactor(particle, gps_pose, gps_w);
      }

      if (use_gps_heading_)
      {
        gpsHeadingFactor(particle, gps_heading, gps_heading_w);
      }

      if (use_imu_)
      {
        imuFactor(particle, imu_pose, imu_w);
      }

      float m_corner_w = (corner_w > 0.) ? corner_w : static_cast<float>(1.);
      float m_planar_w = (planar_w > 0.) ? planar_w : static_cast<float>(1.);
      float m_ground_w = (ground_w > 0.) ? ground_w : static_cast<float>(1.);
      float m_planes_w = (planes_w > 0.) ? planes_w : static_cast<float>(1.);
      float m_gps_w = (gps_w > 0.) ? gps_w : static_cast<float>(1.);
      float m_gps_heading_w = (gps_heading_w > 0.) ? gps_heading_w : static_cast<float>(1.);
      float m_imu_w = (imu_w > 0.) ? imu_w : static_cast<float>(1.);

      particle.w_ = m_corner_w * m_planar_w * m_ground_w * m_planes_w * m_gps_w * m_gps_heading_w * m_imu_w;

      w_sum_ += particle.w_;
#if NUM_THREADS > 1
    });
#endif
  }

#if NUM_THREADS > 1
  thread_pool_->wait();
#endif
}

void PF::update(const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane, const Pose& gps_pose, const double& gps_heading, const Pose& imu_pose, TopologicalMap* topological_map)
{
  std::vector<SemiPlane> filtered_planes;
  if (use_vertical_planes_ || use_ground_plane_)
  {
    preprocessPlanes(prev_average_pose_, topological_map->planes_, filtered_planes);
  }

  for (auto& particle : particles_)
  {
    float ground_w = 0, planes_w = 0, gps_w = 0, gps_heading_w = 0, imu_w = 0;

    if (use_vertical_planes_)
    {
      planesFactor(particle, planes, filtered_planes, planes_w);
    }

    if (use_ground_plane_)
    {
      planesFactor(particle, { ground_plane }, filtered_planes, ground_w);
    }

    if (use_gps_)
    {
      gpsFactor(particle, gps_pose, gps_w);
    }

    if (use_gps_heading_)
    {
      gpsHeadingFactor(particle, gps_heading, gps_heading_w);
    }

    if (use_imu_)
    {
      imuFactor(particle, imu_pose, imu_w);
    }

    float m_particle_w = (particle.w_ > 0.) ? particle.w_ : static_cast<float>(1.);
    float m_ground_w = (ground_w > 0.) ? ground_w : static_cast<float>(1.);
    float m_planes_w = (planes_w > 0.) ? planes_w : static_cast<float>(1.);
    float m_gps_w = (gps_w > 0.) ? gps_w : static_cast<float>(1.);
    float m_gps_heading_w = (gps_heading_w > 0.) ? gps_heading_w : static_cast<float>(1.);
    float m_imu_w = (imu_w > 0.) ? imu_w : static_cast<float>(1.);

    particle.w_ = m_particle_w * m_ground_w * m_planes_w * m_gps_w * m_gps_heading_w * m_imu_w;

    w_sum_ += particle.w_;
  }
}

void PF::gpsFactor(const Particle& particle, const Pose& gps_pose, float& w)
{
  // - GPS [x, y, z] weight
  float dist;
  if (use_gps_altitude_)
  {
    dist = particle.p_.distance(gps_pose);
  }
  else
  {
    dist = particle.p_.distanceXY(gps_pose);
  }
  w = (normalizer_gps_ * static_cast<float>(std::exp(-1. / sigma_gps_ * dist)));
}

void PF::gpsHeadingFactor(const Particle& particle, const double& gps_heading, float& w)
{
  // - GNSS Heading [yaw] weight
  float delta_Y = std::fabs(Const::normalizeAngle(particle.p_.Y_ - gps_heading));

  w = (normalizer_gps_heading_ * static_cast<float>(std::exp(-1. / sigma_gps_heading_ * delta_Y)));
}

void PF::imuFactor(const Particle& particle, const Pose& imu_pose, float& w)
{
  // - IMU [roll, pitch] weight
  float delta_R = std::fabs(Const::normalizeAngle(particle.p_.R_ - imu_pose.R_));
  float delta_P = std::fabs(Const::normalizeAngle(particle.p_.P_ - imu_pose.P_));
  // float delta_Y = std::fabs(Const::normalizeAngle(particle.p_.Y_ - imu_pose.Y_));

  w = (normalizer_imu_ * static_cast<float>(std::exp(-1. / sigma_imu_ * delta_R))) * (normalizer_imu_ * static_cast<float>(std::exp(-1. / sigma_imu_ * delta_P)));  // *
                                                                                                                                                                    //(normalizer_imu_ * static_cast<float>(std::exp(-1. / sigma_imu_ * delta_Y)));
}

void PF::cornersFactor(const Particle& particle, const std::vector<Corner>& corners, OccupancyMap* grid_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D corner map fitting
  // ------------------------------------------------------
  float w_corners = 0;
  for (const auto& corner : corners)
  {
    // Convert feature to the map's referential frame
    Point X = corner.pos_ * particle.tf_;

    // Check cell data
    Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
    if (c->data == nullptr)
    {
      continue;
    }
    std::vector<Corner>* l_corners = c->data->corner_features_;
    if (l_corners == nullptr)
    {
      continue;
    }

    // Search for a correspondence in the current cell first
    Point best_correspondence_point;
    float best_correspondence = 0.5;
    bool found = false;
    for (const auto& l_corner : *l_corners)
    {
      float dist_sq = ((X.x_ - l_corner.pos_.x_) * (X.x_ - l_corner.pos_.x_) + (X.y_ - l_corner.pos_.y_) * (X.y_ - l_corner.pos_.y_) + (X.z_ - l_corner.pos_.z_) * (X.z_ - l_corner.pos_.z_));

      if (dist_sq < best_correspondence)
      {
        best_correspondence_point = l_corner.pos_;
        best_correspondence = dist_sq;
        found = true;
      }
    }

    // Save distance if a correspondence was found
    if (found)
    {
      w_corners += (normalizer_corner_ * static_cast<float>(std::exp(-1. / sigma_corner_matching_ * best_correspondence)));
    }
  }

  w = w_corners;
}

void PF::cornersFactor(const Particle& particle, const std::vector<Corner>& corners, TopologicalMap* topological_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D corner map fitting
  // ------------------------------------------------------
  float w_corners = 0;
  for (const auto& corner : corners)
  {
    // Convert feature to the map's referential frame
    Point X = corner.pos_ * particle.tf_;

    // Get neighbor features
    std::vector<Corner> l_corners = topological_map->getCorners(X.x_, X.y_, X.z_);

    // Search for a correspondence in the current cell first
    Point best_correspondence_point;
    float best_correspondence = 0.5;
    bool found = false;
    for (const auto& l_corner : l_corners)
    {
      float dist_sq = ((X.x_ - l_corner.pos_.x_) * (X.x_ - l_corner.pos_.x_) + (X.y_ - l_corner.pos_.y_) * (X.y_ - l_corner.pos_.y_) + (X.z_ - l_corner.pos_.z_) * (X.z_ - l_corner.pos_.z_));

      if (dist_sq < best_correspondence)
      {
        best_correspondence_point = l_corner.pos_;
        best_correspondence = dist_sq;
        found = true;
      }
    }

    // Save distance if a correspondence was found
    if (found)
    {
      w_corners += (normalizer_corner_ * static_cast<float>(std::exp(-1. / sigma_corner_matching_ * best_correspondence)));
    }
  }

  w = w_corners;
}

void PF::planarsFactor(const Particle& particle, const std::vector<Planar>& planars, OccupancyMap* grid_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  float w_planars = 0.;
  for (const auto& planar : planars)
  {
    // Convert feature to the map's referential frame
    Point X = planar.pos_ * particle.tf_;

    // Check cell data
    Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
    if (c->data == nullptr)
    {
      continue;
    }
    std::vector<Planar>* l_planars = c->data->planar_features_;
    if (l_planars == nullptr)
    {
      continue;
    }

    // Search for a correspondence in the current cell first
    Point best_correspondence_point;
    float best_correspondence = 0.5;
    bool found = false;
    for (const auto& l_planar : *l_planars)
    {
      float dist_sq = ((X.x_ - l_planar.pos_.x_) * (X.x_ - l_planar.pos_.x_) + (X.y_ - l_planar.pos_.y_) * (X.y_ - l_planar.pos_.y_) + (X.z_ - l_planar.pos_.z_) * (X.z_ - l_planar.pos_.z_));

      if (dist_sq < best_correspondence)
      {
        best_correspondence_point = l_planar.pos_;
        best_correspondence = dist_sq;
        found = true;
      }
    }

    // Save distance if a correspondence was found
    if (found)
    {
      w_planars += (normalizer_planar_ * static_cast<float>(std::exp((-1. / sigma_planar_matching_) * best_correspondence)));
    }
  }

  w = w_planars;
}

void PF::planarsFactor(const Particle& particle, const std::vector<Planar>& planars, TopologicalMap* topological_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  float w_planars = 0.;
  for (const auto& planar : planars)
  {
    // Convert feature to the map's referential frame
    Point X = planar.pos_ * particle.tf_;

    // Check cell data
    std::vector<Planar> l_planars = topological_map->getPlanars(X.x_, X.y_, X.z_);

    // Search for a correspondence in the current cell first
    Point best_correspondence_point;
    float best_correspondence = 0.5;
    bool found = false;
    for (const auto& l_planar : l_planars)
    {
      float dist_sq = ((X.x_ - l_planar.pos_.x_) * (X.x_ - l_planar.pos_.x_) + (X.y_ - l_planar.pos_.y_) * (X.y_ - l_planar.pos_.y_) + (X.z_ - l_planar.pos_.z_) * (X.z_ - l_planar.pos_.z_));

      if (dist_sq < best_correspondence)
      {
        best_correspondence_point = l_planar.pos_;
        best_correspondence = dist_sq;
        found = true;
      }
    }

    // Save distance if a correspondence was found
    if (found)
    {
      w_planars += (normalizer_planar_ * static_cast<float>(std::exp((-1. / sigma_planar_matching_) * best_correspondence)));
    }
  }

  w = w_planars;
}

void PF::cornersFactorSimple(const Particle& particle, const std::vector<Corner>& corners, OccupancyMap* grid_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D corner map fitting
  // ------------------------------------------------------
  float w_corners = 0;
  for (const auto& corner : corners)
  {
    // Convert feature to the map's referential frame
    Point X = corner.pos_ * particle.tf_;

    // Check cell data
    Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
    if (c->data == nullptr)
    {
      continue;
    }
    std::vector<Corner>* l_corners = c->data->corner_features_;
    if (l_corners == nullptr)
    {
      continue;
    }

    w_corners += !(l_corners->empty());
  }

  sigma_corner_matching_ = static_cast<float>(corners.size()) / simple_weight_factor_;
  normalizer_corner_ = static_cast<float>(1.) / (sigma_corner_matching_ * std::sqrt(M_2PI));
  w = (normalizer_corner_ * static_cast<float>(std::exp((1 / sigma_corner_matching_) * w_corners)));
}

void PF::cornersFactorSimple(const Particle& particle, const std::vector<Corner>& corners, TopologicalMap* topological_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D corner map fitting
  // ------------------------------------------------------
  float w_corners = 0;
  for (const auto& corner : corners)
  {
    // Convert feature to the map's referential frame
    Point X = corner.pos_ * particle.tf_;

    // Get neighbor features
    std::vector<Corner> l_corners = topological_map->getCorners(X.x_, X.y_, X.z_);

    w_corners += !(l_corners.empty());
  }

  sigma_corner_matching_ = static_cast<float>(corners.size()) / simple_weight_factor_;
  normalizer_corner_ = static_cast<float>(1.) / (sigma_corner_matching_ * std::sqrt(M_2PI));
  w = (normalizer_corner_ * static_cast<float>(std::exp((1 / sigma_corner_matching_) * w_corners)));
}

void PF::planarsFactorSimple(const Particle& particle, const std::vector<Planar>& planars, OccupancyMap* grid_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  float w_planars = 0.;
  for (const auto& planar : planars)
  {
    // Convert feature to the map's referential frame
    Point X = planar.pos_ * particle.tf_;

    // Check cell data
    Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
    if (c->data == nullptr)
    {
      continue;
    }
    std::vector<Planar>* l_planars = c->data->planar_features_;
    if (l_planars == nullptr)
    {
      continue;
    }

    w_planars += !(l_planars->empty());
  }

  sigma_planar_matching_ = static_cast<float>(planars.size()) / simple_weight_factor_;
  normalizer_planar_ = static_cast<float>(1.) / (sigma_planar_matching_ * std::sqrt(M_2PI));
  w = (normalizer_planar_ * static_cast<float>(std::exp((1 / sigma_planar_matching_) * w_planars)));
}

void PF::planarsFactorSimple(const Particle& particle, const std::vector<Planar>& planars, TopologicalMap* topological_map, float& w)
{
  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  float w_planars = 0.;
  for (const auto& planar : planars)
  {
    // Convert feature to the map's referential frame
    Point X = planar.pos_ * particle.tf_;

    // Check cell data
    std::vector<Planar> l_planars = topological_map->getPlanars(X.x_, X.y_, X.z_);

    w_planars += !(l_planars.empty());
  }

  sigma_planar_matching_ = static_cast<float>(planars.size()) / simple_weight_factor_;
  normalizer_planar_ = static_cast<float>(1.) / (sigma_planar_matching_ * std::sqrt(M_2PI));
  w = (normalizer_planar_ * static_cast<float>(std::exp((1 / sigma_planar_matching_) * w_planars)));
}

void PF::preprocessPlanes(const Pose& robot_pose, const std::vector<SemiPlane>& planes, std::vector<SemiPlane>& filtered_planes)
{
  // Stabilize the global planes array by considering only the ones that have their closest extrema to the robot closer
  // than a given valid range
  double valid_range = 10.0;
  Point robot_position = robot_pose.getXYZ();

  size_t size = planes.size();
  for (size_t i = 0; i < size; i++)
  {
    size_t extremas_size = planes[i].extremas_.size();
    double min_dist = std::numeric_limits<double>::max();
    for (size_t j = 0; j < extremas_size; j++)
    {
      double dist = planes[i].extremas_[j].distance(robot_position);
      if (dist < min_dist)
      {
        min_dist = dist;
      }
    }

    if (min_dist < valid_range)
    {
      filtered_planes.push_back(planes[i]);
    }
  }
}

void PF::planesFactor(const Particle& particle, const std::vector<SemiPlane>& planes, const std::vector<SemiPlane>& global_planes, float& w)
{
  float w_planes = 0.;
  // ----------------------------------------------------------------------------
  // ------ Search for correspondences between local planes and global planes
  // ------ Three stage process:
  // ------  * (A) Check semi-plane overlap
  // ------  * (B) Compare planes normals
  // ------  *  If (B), then check (C) plane to plane distance
  // ----------------------------------------------------------------------------

  // Define correspondence thresholds
  float v_dist = 0.2;   // max vector displacement for all the components
  float sp_dist = 0.2;  // max distance from source plane centroid to target plane
  float area_th = 2.0;  // minimum overlapping area between semiplanes

  // Correspondence result
  float correspondence_vec;
  float correspondence_centroid;

  for (const auto& plane : planes)
  {
    if (plane.points_.empty())
    {
      continue;
    }

    // Initialize correspondence deltas
    float vec_disp = v_dist;
    float point2plane = sp_dist;
    float ov_area = area_th;

    // Convert local plane to maps' referential frame
    SemiPlane l_plane = plane;
    for (auto& point : l_plane.points_)
    {
      point = point * particle.tf_;  // Convert plane points
    }
    for (auto& point : l_plane.extremas_)
    {
      point = point * particle.tf_;  // Convert plane boundaries
    }
    l_plane.centroid_ = l_plane.centroid_ * particle.tf_;  // Convert the centroid
    Ransac::estimateNormal(l_plane.points_, l_plane.a_, l_plane.b_, l_plane.c_,
                           l_plane.d_);  // Convert plane normal

    bool found = false;
    for (auto& g_plane : global_planes)
    {
      if (g_plane.points_.empty())
      {
        continue;
      }

      // --------------------------------
      // (A) - Check semi-plane overlap
      // --------------------------------

      // First project the global and local plane extremas to the global plane reference frame
      Tf ref_frame = g_plane.inv_local_ref_;
      SemiPlane gg_plane;
      SemiPlane lg_plane;
      for (const auto& extrema : g_plane.extremas_)
      {
        Point p = extrema * ref_frame;
        p.z_ = 0;
        gg_plane.extremas_.push_back(p);
      }
      for (const auto& extrema : l_plane.extremas_)
      {
        Point p = extrema * ref_frame;
        p.z_ = 0;
        lg_plane.extremas_.push_back(p);
      }

      // Now, check for transformed polygon intersections
      SemiPlane isct;
      ConvexHull::polygonIntersection(gg_plane, lg_plane, isct.extremas_);

      // Compute the intersection semi plane area
      isct.setArea();

      if (isct.area_ > ov_area)
      {
        // --------------------------------
        // (B) - Compare plane normals
        // --------------------------------

        Vec u(l_plane.a_, l_plane.b_, l_plane.c_);
        Vec v(g_plane.a_, g_plane.b_, g_plane.c_);

        float D = ((u - v).norm3D() < (u + v).norm3D()) ? (u - v).norm3D() : (u + v).norm3D();

        // Check if normal vectors match
        if (D < vec_disp)
        {
          // --------------------------------
          // (C) - Compute local plane centroid distance to global plane
          // --------------------------------
          float l_point2plane = g_plane.point2Plane(l_plane.centroid_);
          if (l_point2plane < point2plane)
          {
            // We found a correspondence, so, we must save the correspondence deltas
            vec_disp = D;
            point2plane = l_point2plane;
            ov_area = isct.area_;

            // Save correspondence errors
            correspondence_vec = D;
            correspondence_centroid = l_point2plane;

            // Set correspondence flag
            found = true;
          }
        }
      }
    }

    if (found)
    {
      w_planes += ((normalizer_plane_vector_ * static_cast<float>(std::exp((-1. / sigma_plane_matching_vector_) * correspondence_vec))) *
                   (normalizer_plane_centroid_ * static_cast<float>(std::exp((-1. / sigma_plane_matching_centroid_) * correspondence_centroid))));
    }
  }

  w = w_planes;
}

void PF::normalizeWeights()
{
  if (w_sum_ > 0.)
  {
    for (auto& particle : particles_)
    {
      particle.w_ /= w_sum_;
    }
  }
  else
  {
    for (auto& particle : particles_)
    {
      particle.w_ = static_cast<float>(1.) / static_cast<float>(particles_size_);
    }
  }
}

void PF::resample()
{
  float cweight = 0.;
  uint32_t n = particles_size_;

  // - Compute the cumulative weights
  for (const auto& p : particles_)
    cweight += p.w_;
  // - Compute the interval
  float interval = cweight / n;
  // - Compute the initial target weight
  auto target = static_cast<float>(interval * ::drand48());

  // - Compute the resampled indexes
  cweight = 0.;
  std::vector<uint32_t> indexes(n);
  n = 0.;
  uint32_t i = 0;

  for (const auto& p : particles_)
  {
    cweight += p.w_;
    while (cweight > target)
    {
      indexes[n++] = i;
      target += interval;
    }

    i++;
  }

  // - Update particle set
  for (size_t j = 0; j < indexes.size(); j++)
  {
    particles_[j].pp_ = particles_[indexes[j]].pp_;
    particles_[j].ptf_ = particles_[indexes[j]].ptf_;
    particles_[j].p_ = particles_[indexes[j]].p_;
    particles_[j].tf_ = particles_[indexes[j]].tf_;
    particles_[j].w_ = particles_[indexes[j]].w_;
    particles_[j].which_cluster_ = particles_[indexes[j]].which_cluster_;
  }
}

void PF::measurePrecision(const vineslam::Pose& robot_pose, const std::vector<Corner>& corners, vineslam::OccupancyMap* grid_map, uint32_t& precision)
{
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  double w_corners = 0;
  double total_corners = 0;
  for (const auto& corner : corners)
  {
    if (corner.pos_.norm3D() < 20.0)
    {
      // Convert feature to the map's referential frame
      Point X = corner.pos_ * tf;

      // Check cell data
      Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
      if (c->data == nullptr)
      {
        continue;
      }
      std::vector<Corner>* l_corners = c->data->corner_features_;
      if (l_corners == nullptr)
      {
        continue;
      }

      w_corners += !(l_corners->empty());
      total_corners += 1.0;
    }
  }

  precision = (uint32_t)((w_corners / total_corners) * 100.0);
}

void PF::measurePrecision(const vineslam::Pose& robot_pose, const std::vector<Corner>& corners, vineslam::TopologicalMap* topological_map, uint32_t& precision)
{
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  double w_corners = 0;
  double total_corners = 0;
  for (const auto& corner : corners)
  {
    if (corner.pos_.norm3D() < 20.0)
    {
      // Convert feature to the map's referential frame
      Point X = corner.pos_ * tf;

      std::vector<Corner> l_corners = topological_map->getCorners(X.x_, X.y_, X.z_);
      w_corners += !(l_corners.empty());
      total_corners += 1.0;
    }
  }

  precision = (uint32_t)((w_corners / total_corners) * 100.0);
}

void PF::measurePrecision(const vineslam::Pose& robot_pose, const std::vector<Planar>& planars, vineslam::OccupancyMap* grid_map, uint32_t& precision)
{
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // To speed up the process, we only use 1000 featues to measure precision
  uint32_t n_features = 1000;
  uint32_t iterator = (planars.size() > n_features) ? (planars.size() / n_features) : 1;

  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  double w_planars = 0;
  double total_planars = 0;
  for (size_t i = 0; i < planars.size(); i += iterator)
  {
    if (planars[i].pos_.norm3D() < 20.0)
    {
      // Convert feature to the map's referential frame
      Point X = planars[i].pos_ * tf;

      // Check cell data
      Cell* c = &(*grid_map)(X.x_, X.y_, X.z_);
      if (c->data == nullptr)
      {
        continue;
      }
      std::vector<Planar>* l_planars = c->data->planar_features_;
      if (l_planars == nullptr)
      {
        continue;
      }

      w_planars += !(l_planars->empty());
      total_planars += 1.0;
    }
  }

  precision = (uint32_t)((w_planars / total_planars) * 100.0);
}

void PF::measurePrecision(const vineslam::Pose& robot_pose, const std::vector<Planar>& planars, vineslam::TopologicalMap* topological_map, uint32_t& precision)
{
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // To speed up the process, we only use 1000 featues to measure precision
  uint32_t n_features = 1000;
  uint32_t iterator = (planars.size() > n_features) ? (planars.size() / n_features) : 1;

  // ------------------------------------------------------
  // --- 3D planar map fitting
  // ------------------------------------------------------
  double w_planars = 0;
  double total_planars = 0;
  for (size_t i = 0; i < planars.size(); i += iterator)
  {
    if (planars[i].pos_.norm3D() < 20.0)
    {
      // Convert feature to the map's referential frame
      Point X = planars[i].pos_ * tf;

      std::vector<Planar> l_planars = topological_map->getPlanars(X.x_, X.y_, X.z_);
      w_planars += !(l_planars.empty());
      total_planars += 1.0;
    }
  }

  precision = (uint32_t)((w_planars / total_planars) * 100.0);
}

}  // namespace vineslam
