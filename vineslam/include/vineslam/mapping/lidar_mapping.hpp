#pragma once

#include <iostream>
#include <unordered_map>
#include <deque>
#include <eigen3/Eigen/Dense>

#include <vineslam/params.hpp>
#include <vineslam/feature/three_dimensional.hpp>
#include <vineslam/mapping/topological_map.hpp>
#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/mapping/elevation_map.hpp>
#include <vineslam/math/Point.hpp>
#include <vineslam/math/Pose.hpp>
#include <vineslam/math/Tf.hpp>
#include <vineslam/math/Const.hpp>
#include <vineslam/filters/convex_hull.hpp>
#include <vineslam/utils/Timer.hpp>
#include <vineslam/extern/thread_pool.h>

#ifdef USE_CUDA
#include <vineslam/filters/ransac_cu.hpp>
#else
#include <vineslam/filters/ransac.hpp>
#endif

namespace vineslam
{
class LidarMapper
{
public:
  // Class constructor - receives and saves the system
  // parameters
  explicit LidarMapper();

  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, OccupancyMap& grid_map,
                    ElevationMap& elevation_map);
  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, OccupancyMap& grid_map);
  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, TopologicalMap& topological_map,
                    ElevationMap& elevation_map);
  void registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const std::vector<SemiPlane>& planes, const SemiPlane& ground, TopologicalMap& topological_map);

  void setLaser2Base(const float& x, const float& y, const float& z, const float& roll, const float& pitch,
                     const float& yaw)
  {
    laser2base_x_ = x;
    laser2base_y_ = y;
    laser2base_z_ = z;
    laser2base_roll_ = roll;
    laser2base_pitch_ = pitch;
    laser2base_yaw_ = yaw;
  }

  // Parameters
  Parameters params_;

  // Class profiler
  Timer* t_;

  // Previous robot pose
  Pose prev_robot_pose_;

  // Robot dimensions vars
  float robot_dim_x_;
  float robot_dim_y_;
  float robot_dim_z_;

  // Mapper iterator
  int it_{};
  // Plane filter frequency
  int filter_frequency_{};

  // Transformation parameters
  float laser2base_x_{}, laser2base_y_{}, laser2base_z_{}, laser2base_roll_{}, laser2base_pitch_{}, laser2base_yaw_{};

  // Thread pool
  lama::ThreadPool* thread_pool_;

private:
  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Adds the corner features to the global map
  static void globalCornerMap(const Pose& robot_pose, const std::vector<Corner>& corners, OccupancyMap& grid_map);
  static void globalCornerMap(const Pose& robot_pose, const std::vector<Corner>& corners,
                              TopologicalMap& topological_map);
  // Adds the planar features to the global map
  static void globalPlanarMap(const Pose& robot_pose, const std::vector<Planar>& planars, OccupancyMap& grid_map);
  static void globalPlanarMap(const Pose& robot_pose, const std::vector<Planar>& planars,
                              TopologicalMap& topological_map);
  // Adds the plane features to the global map
  void globalPlaneMap(const Pose& robot_pose, const std::vector<SemiPlane>& planes, OccupancyMap& grid_map);
  void globalPlaneMap(const Pose& robot_pose, const std::vector<SemiPlane>& planes, TopologicalMap& topological_map);
  // Adds new altemetry measures to the elevation map
  void globalElevationMap(const Pose& robot_pose, const Plane& ground_plane, ElevationMap& elevation_map);
};
}  // namespace vineslam