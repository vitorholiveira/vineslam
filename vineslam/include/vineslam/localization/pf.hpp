#pragma once

// Include class objects
#include <vineslam/params.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/topological_map.hpp>
#include <vineslam/interface/localization_mapping_interface.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/math/Stat.hpp>
#include <vineslam/filters/convex_hull.hpp>
#include <vineslam/utils/Timer.hpp>
#include <vineslam/extern/thread_pool.h>

#ifdef USE_CUDA
#include <vineslam/filters/ransac_cu.hpp>
#else
#include <vineslam/filters/ransac.hpp>
#endif

// Include std members
#include <cstdlib>
#include <limits>
#include <chrono>
#include <iostream>
#include <map>
#include <cmath>

namespace vineslam
{
// Struct that represents a single particle with
// - identification number
// - 6-DOF pose
// - weight
struct Particle
{
  Particle() = default;
  Particle(const int& id, const Pose& p, const float& w)
  {
    (*this).id_ = id;
    (*this).p_ = p;
    (*this).w_ = w;

    std::array<float, 9> Rot{};
    p.toRotMatrix(Rot);
    std::array<float, 3> trans = { p.x_, p.y_, p.z_ };
    tf_ = Tf(Rot, trans);
  }

  int id_{};
  Pose p_;   // pose
  Pose pp_;  // previous pose
  Tf tf_;    // pose resented as an homogeneous transformation
  Tf ptf_;   // previous homogeneous transformation
  float w_{};
  int which_cluster_{};
};

class PF
{
public:
  // Class constructor
  // - initializes the total set of particles
  PF(const Parameters& params, const Pose& initial_pose);

  // Apply odometry motion model to all particles
  void motionModel(const Pose& odom_inc);

  // Update particles weights using the multi-layer map
  void update(const std::vector<Corner>& corners, const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane,
              const Pose& gps_pose, const double& gps_heading, const Pose& imu_pose, OccupancyMap* grid_map);
  void update(const std::vector<Corner>& corners, const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane,
              const Pose& gps_pose, const double& gps_heading, const Pose& imu_pose, TopologicalMap* topological_map);
  void update(const std::vector<SemiPlane>& planes, const SemiPlane& ground_plane, const Pose& gps_pose, const double& gps_heading, const Pose& imu_pose,
              TopologicalMap* topological_map);

  // Normalize particles weights
  void normalizeWeights();
  // Resample particles
  void resample();

  // Measure localization precision (%)
  void measurePrecision(const Pose& robot_pose, const std::vector<Corner>& corners, OccupancyMap* grid_map, uint32_t& precision);
  void measurePrecision(const Pose& robot_pose, const std::vector<Corner>& corners, TopologicalMap* topological_map, uint32_t& precision);
  void measurePrecision(const Pose& robot_pose, const std::vector<Planar>& planars, OccupancyMap* grid_map, uint32_t& precision);
  void measurePrecision(const Pose& robot_pose, const std::vector<Planar>& planars, TopologicalMap* topological_map, uint32_t& precision);

  // Particle weight sum
  float w_sum_{};

  // Particles
  std::vector<Particle> particles_;

  // Average pose calculated in the last iteration
  // We use it in the preprocessing algorithm for semimplanes
  Pose prev_average_pose_;

  // Thread pool
  lama::ThreadPool* thread_pool_;

  // Profiler
  Timer* t_;

  // Observations to use
  bool use_lidar_features_;
  bool use_vertical_planes_;
  bool use_ground_plane_;
  bool use_gps_;
  bool use_gps_altitude_;
  bool use_gps_heading_;
  bool use_imu_;

  // Filter settings
  float sigma_landmark_matching_;
  float sigma_feature_matching_;
  float sigma_corner_matching_;
  float sigma_planar_matching_;
  float sigma_plane_matching_vector_;
  float sigma_plane_matching_centroid_;
  float sigma_gps_;
  float sigma_gps_heading_;
  float sigma_imu_;
  float simple_weight_factor_;

  // Normalizers
  float normalizer_plane_vector_;
  float normalizer_plane_centroid_;
  float normalizer_planar_;
  float normalizer_corner_;
  float normalizer_imu_;
  float normalizer_gps_;
  float normalizer_gps_heading_;

private:
  // Samples a zero-mean gaussian distribution with a given standard deviation
  float sampleGaussian(const float& sigma, const unsigned long int& S = 0);

  // Medium level corner features layer
  void cornersFactor(const Particle& particle, const std::vector<Corner>& corners, OccupancyMap* grid_map, float& w);
  void cornersFactor(const Particle& particle, const std::vector<Corner>& corners, TopologicalMap* topological_map, float& w);
  void cornersFactorSimple(const Particle& particle, const std::vector<Corner>& corners, OccupancyMap* grid_map, float& w);
  void cornersFactorSimple(const Particle& particle, const std::vector<Corner>& corners, TopologicalMap* topological_map, float& w);

  // Medium level planar features layer
  void planarsFactor(const Particle& particle, const std::vector<Planar>& planars, OccupancyMap* grid_map, float& w);
  void planarsFactor(const Particle& particle, const std::vector<Planar>& planars, TopologicalMap* topological_map, float& w);
  void planarsFactorSimple(const Particle& particle, const std::vector<Planar>& planars, OccupancyMap* grid_map, float& w);
  void planarsFactorSimple(const Particle& particle, const std::vector<Planar>& planars, TopologicalMap* topological_map, float& w);

  // Medium ground plane layer
  void preprocessPlanes(const Pose& robot_pose, const std::vector<SemiPlane>& planes, std::vector<SemiPlane>& filtered_planes);
  void planesFactor(const Particle& particle, const std::vector<SemiPlane>& planes, const std::vector<SemiPlane>& global_planes, float& w);

  // GPS
  void gpsFactor(const Particle& particle, const Pose& gps_pose, float& w);
  // GPS heading
  void gpsHeadingFactor(const Particle& particle, const double& gps_heading, float& w);
  // IMU
  void imuFactor(const Particle& particle, const Pose& imu_pose, float& w);

  // Number of particles
  uint32_t particles_size_;

  // Parameters structure
  Parameters params_;
};

}  // namespace vineslam
