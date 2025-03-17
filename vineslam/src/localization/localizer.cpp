#include "../../include/vineslam/localization/localizer.hpp"

namespace vineslam
{

Localizer::Localizer(Parameters params) : params_(std::move(params))
{
}

void Localizer::init(const Pose& initial_pose, LocalizationMappingInterface* localization_mapping_interface)
{
  // Initialize the particle filter
  pf_ = new PF(params_, initial_pose);
  pf_->t_ = t_;

  // Initialize localization and mapping interface
  localization_mapping_interface_ = localization_mapping_interface;

  // Compute average pose and standard deviation of the
  // first distribution
  std::vector<Pose> poses;
  for (auto& particle : pf_->particles_)
  {
    poses.push_back(particle.p_);
  }
  average_pose_ = Pose(poses);
  last_update_pose_ = initial_pose;

  p_odom_ = initial_pose;
  init_flag_ = true;
}

void Localizer::process(const Pose& wheel_odom_inc, const Observation& obsv, OccupancyMap* grid_map)
{
  // Resets
  pf_->w_sum_ = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Predict the robot motion
  // ------------------------------------------------------------------------------
  Pose odom_inc = wheel_odom_inc;
  odom_inc.normalize();

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
#if VERBOSE == 1
  t_->tick("pf::motionModel()");
#endif
  pf_->motionModel(odom_inc);
#if VERBOSE == 1
  t_->tock();
#endif

  Pose odom = p_odom_ + odom_inc;
  Pose delta_pose = odom - last_update_pose_;
  delta_pose.normalize();

  if (std::fabs(delta_pose.x_) > 0.15 || std::fabs(delta_pose.y_) > 0.15 || std::fabs(delta_pose.Y_) > 3 * DEGREE_TO_RAD)  // || init_flag_)
  {
    // ------------------------------------------------------------------------------
    // ---------------- Update particles weights using multi-layer map
    // ------------------------------------------------------------------------------
#if VERBOSE == 1
    t_->tick("pf::update()");
#endif
    pf_->update(obsv.corners_, obsv.planars_, obsv.planes_, obsv.ground_plane_, obsv.gps_pose_, obsv.gps_heading_, obsv.imu_pose_, grid_map);
#if VERBOSE == 1
    t_->tock();
#endif

    // ------------------------------------------------------------------------------
    // ---------------- Normalize particle weights
    // ------------------------------------------------------------------------------
#if VERBOSE == 1
    t_->tick("pf::normalizeWeights()");
#endif
    pf_->normalizeWeights();
#if VERBOSE == 1
    t_->tock();
#endif

    // ------------------------------------------------------------------------------
    // ---------------- Resample particles
    // ------------------------------------------------------------------------------
#if VERBOSE == 1
    t_->tick("pf::resample()");
#endif
    pf_->resample();
#if VERBOSE == 1
    t_->tock();
#endif

    last_update_pose_ = odom;
    init_flag_ = false;
  }

  // - Compute final robot pose using the mean of the particles poses
#if VERBOSE == 1
  t_->tick("localizer::average()");
#endif
  std::vector<Pose> poses;
  for (const auto& particle : pf_->particles_)
  {
    poses.push_back(particle.p_);
  }
  average_pose_ = Pose(poses);
  pf_->prev_average_pose_ = average_pose_;
#if VERBOSE == 1
  t_->tock();
#endif

  // - Save current control to use in the next iteration
  p_odom_ = odom;
}

void Localizer::process(const Pose& wheel_odom_inc, const Observation& obsv, TopologicalMap* topological_map)
{
  // Resets
  pf_->w_sum_ = 0.;

  // ------------------------------------------------------------------------------
  // ---------------- Predict the robot motion
  // ------------------------------------------------------------------------------
  Pose odom_inc = wheel_odom_inc;
  odom_inc.normalize();

  // ------------------------------------------------------------------------------
  // ---------------- Draw particles using odometry motion model
  // ------------------------------------------------------------------------------
#if VERBOSE == 1
  t_->tick("pf::motionModel()");
#endif
  pf_->motionModel(odom_inc);
#if VERBOSE == 1
  t_->tock();
#endif

  Pose odom = p_odom_ + odom_inc;
  Pose delta_pose = odom - last_update_pose_;
  delta_pose.normalize();

  if (std::fabs(delta_pose.x_) > 0.05 || std::fabs(delta_pose.y_) > 0.05 || std::fabs(delta_pose.Y_) > 1 * DEGREE_TO_RAD)  // || init_flag_)
  {
    // ------------------------------------------------------------------------------
    // ---------------- Update particles weights using multi-layer map
    // ------------------------------------------------------------------------------
#if VERBOSE == 1
    t_->tick("pf::update()");
#endif
    pf_->update(obsv.corners_, obsv.planars_, obsv.planes_, obsv.ground_plane_, obsv.gps_pose_, obsv.gps_heading_, obsv.imu_pose_, topological_map);
#if VERBOSE == 1
    t_->tock();
#endif

    // ------------------------------------------------------------------------------
    // ---------------- Normalize particle weights
    // ------------------------------------------------------------------------------
#if VERBOSE == 1
    t_->tick("pf::normalizeWeights()");
#endif
    pf_->normalizeWeights();
#if VERBOSE == 1
    t_->tock();
#endif

    // ------------------------------------------------------------------------------
    // ---------------- Resample particles
    // ------------------------------------------------------------------------------
#if VERBOSE == 1
    t_->tick("pf::resample()");
#endif
    pf_->resample();
#if VERBOSE == 1
    t_->tock();
#endif

    last_update_pose_ = odom;
    init_flag_ = false;
  }

  // - Compute final robot pose using the mean of the particles poses
#if VERBOSE == 1
  t_->tick("pf::average()");
#endif
  std::vector<Pose> poses;
  for (const auto& particle : pf_->particles_)
  {
    poses.push_back(particle.p_);
  }
  average_pose_ = Pose(poses);
  pf_->prev_average_pose_ = average_pose_;
#if VERBOSE == 1
  t_->tock();
#endif

  // - Save current control to use in the next iteration
  p_odom_ = odom;
}

Pose Localizer::getPose() const
{
  return average_pose_;
}

void Localizer::getParticles(std::vector<Particle>& in) const
{
  in.resize(pf_->particles_.size());
  for (size_t i = 0; i < in.size(); i++)
    in[i] = pf_->particles_[i];
}

void Localizer::changeGPSFlag(const bool& val)
{
  pf_->use_gps_ = val;
}

uint32_t Localizer::getPrecision(const std::vector<Corner>& corners, vineslam::OccupancyMap* grid_map) const
{
  uint32_t precision;
  pf_->measurePrecision(average_pose_, corners, grid_map, precision);
  return precision;
}

uint32_t Localizer::getPrecision(const std::vector<Corner>& corners, vineslam::TopologicalMap* topological_map) const
{
  uint32_t precision;
  pf_->measurePrecision(average_pose_, corners, topological_map, precision);
  return precision;
}

uint32_t Localizer::getPrecision(const std::vector<Planar>& planars, vineslam::OccupancyMap* grid_map) const
{
  uint32_t precision;
  pf_->measurePrecision(average_pose_, planars, grid_map, precision);
  return precision;
}

uint32_t Localizer::getPrecision(const std::vector<Planar>& planars, vineslam::TopologicalMap* topological_map) const
{
  uint32_t precision;
  pf_->measurePrecision(average_pose_, planars, topological_map, precision);
  return precision;
}

}  // namespace vineslam