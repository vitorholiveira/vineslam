#pragma once

// Class objects
#include <vineslam/params.hpp>
#include <vineslam/feature/visual.hpp>
#include <vineslam/feature/semantic.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/matcher/icp.hpp>
#include <vineslam/localization/pf.hpp>
#include <vineslam/interface/localization_mapping_interface.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Const.hpp>

// std, eigen
#include <iostream>
#include <thread>
#include <map>

namespace vineslam
{
// Structure that stores  observations to use in the localization procedure
struct Observation
{
  std::vector<Planar> planars_;
  std::vector<Corner> corners_;
  std::vector<SemiPlane> planes_;
  SemiPlane ground_plane_;
  Pose gps_pose_;
  double gps_heading_;
  Pose imu_pose_;
};

class Localizer
{
public:
  // Class constructor
  explicit Localizer(Parameters params);

  // Initializes the particle filter with the number of particles
  // and the first odometry pose
  void init(const Pose& initial_pose, LocalizationMappingInterface* localization_mapping_interface);

  // Global function that handles all the localization process
  // Arguments:
  // - wheel_odom_inc: odometry incremental pose
  // - obsv:           current multi-layer mapping observation
  // - grid_map:       occupancy grid map that encodes the multi-layer map information
  // OR
  // - topological_map: topological map that encodes the multi-layer map information
  void process(const Pose& odom, const Observation& obsv, OccupancyMap* grid_map);
  void process(const Pose& odom, const Observation& obsv, TopologicalMap* topological_map);

  // Export the final pose resultant from the localization procedure
  Pose getPose() const;
  // Export the all the poses referent to all the particles
  void getParticles(std::vector<Particle>& in) const;
  // Calculate and export localization precision
  uint32_t getPrecision(const std::vector<Corner>& corners, OccupancyMap* grid_map) const;
  uint32_t getPrecision(const std::vector<Corner>& corners, TopologicalMap* topological_map) const;
  uint32_t getPrecision(const std::vector<Planar>& planars, OccupancyMap* grid_map) const;
  uint32_t getPrecision(const std::vector<Planar>& planars, TopologicalMap* topological_map) const;

  // Setters
  void changeGPSFlag(const bool& val);

  // Particle filter object
  PF* pf_{};

  // Localization and mapping interface
  LocalizationMappingInterface* localization_mapping_interface_;

  // Profiler
  Timer* t_;

private:
  // Average particles pose
  Pose average_pose_;
  Pose last_update_pose_;
  Pose p_odom_;

  // Particles before resampling
  std::vector<Particle> m_particles_;

  // Flags
  bool init_flag_;

  // Input parameters
  Parameters params_;
};

}  // namespace vineslam
