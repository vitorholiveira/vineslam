#include "../../include/vineslam/mapping/lidar_mapping.hpp"

namespace vineslam
{
LidarMapper::LidarMapper()
{
  filter_frequency_ = 30;
  it_ = 0;
}

void LidarMapper::registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners,
                               const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes,
                               const SemiPlane& ground, OccupancyMap& grid_map, ElevationMap& elevation_map)
{
  // - 3D PCL corner map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalCornerMap()");
#endif
  globalCornerMap(robot_pose, corners, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL planar map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlanarMap()");
#endif
  globalPlanarMap(robot_pose, planars, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL plane map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlaneMap()");
#endif
  globalPlaneMap(robot_pose, planes, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif
#if VERBOSE == 1
  t_->tick("lidarMapper::globalGroundMap()");
#endif
  globalPlaneMap(robot_pose, { ground }, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - Elevation map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalElevationMap()");
#endif
  globalElevationMap(robot_pose, ground, elevation_map);
#if VERBOSE == 1
  t_->tock();
#endif

  // Store robot pose to use in the next iteration
  prev_robot_pose_ = robot_pose;

  // Increment mapper iterator
  it_++;
}

void LidarMapper::registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners,
                               const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes,
                               const SemiPlane& ground, TopologicalMap& topological_map, ElevationMap& elevation_map)
{
  // - 3D PCL corner map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalCornerMap()");
#endif
  globalCornerMap(robot_pose, corners, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL planar map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlanarMap()");
#endif
  globalPlanarMap(robot_pose, planars, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL plane map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlaneMap()");
#endif
  globalPlaneMap(robot_pose, planes, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif
#if VERBOSE == 1
  t_->tick("lidarMapper::globalGroundMap()");
#endif
  globalPlaneMap(robot_pose, { ground }, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - Elevation map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalElevationMap()");
#endif
  globalElevationMap(robot_pose, ground, elevation_map);
#if VERBOSE == 1
  t_->tock();
#endif

  // Store robot pose to use in the next iteration
  prev_robot_pose_ = robot_pose;

  // Increment mapper iterator
  it_++;
}

void LidarMapper::registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners,
                               const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes,
                               const SemiPlane& ground, OccupancyMap& grid_map)
{
  // - 3D PCL corner map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalCornerMap()");
#endif
  globalCornerMap(robot_pose, corners, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL planar map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlanarMap()");
#endif
  globalPlanarMap(robot_pose, planars, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL plane map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlaneMap()");
#endif
  globalPlaneMap(robot_pose, planes, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif
#if VERBOSE == 1
  t_->tick("lidarMapper::globalGroundMap()");
#endif
  globalPlaneMap(robot_pose, { ground }, grid_map);
#if VERBOSE == 1
  t_->tock();
#endif

  // Store robot pose to use in the next iteration
  prev_robot_pose_ = robot_pose;

  // Increment mapper iterator
  it_++;
}

void LidarMapper::registerMaps(const Pose& robot_pose, const std::vector<Corner>& corners,
                               const std::vector<Planar>& planars, const std::vector<SemiPlane>& planes,
                               const SemiPlane& ground, TopologicalMap& topological_map)
{
  // - 3D PCL corner map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalCornerMap()");
#endif
  globalCornerMap(robot_pose, corners, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL planar map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlanarMap()");
#endif
  globalPlanarMap(robot_pose, planars, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif
  // - 3D PCL plane map estimation
#if VERBOSE == 1
  t_->tick("lidarMapper::globalPlaneMap()");
#endif
  globalPlaneMap(robot_pose, planes, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif
#if VERBOSE == 1
  t_->tick("lidarMapper::globalGroundMap()");
#endif
  globalPlaneMap(robot_pose, { ground }, topological_map);
#if VERBOSE == 1
  t_->tock();
#endif

  // Store robot pose to use in the next iteration
  prev_robot_pose_ = robot_pose;

  // Increment mapper iterator
  it_++;
}

void LidarMapper::globalCornerMap(const Pose& robot_pose, const std::vector<Corner>& corners, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Local array to store the new planar features
  std::vector<Corner> new_corners;

  // ----------------------------------------------------------------------------
  // ------ Insert corner into the grid map
  // ----------------------------------------------------------------------------
  for (auto& corner : corners)
  {
    // - First convert them to map's referential using the robot pose
    Point l_pt = corner.pos_ * tf;

    // - Then, look for correspondences in the local map
    Corner correspondence{};
    float best_correspondence = 0.20;
    bool found = false;
    std::vector<Corner>* l_corners{ nullptr };

    if (grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data != nullptr)
    {
      l_corners = grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->corner_features_;
    }

    if (l_corners != nullptr)
    {
      for (const auto& l_corner : *l_corners)
      {
        float dist_min = l_pt.distance(l_corner.pos_);

        if (dist_min < best_correspondence)
        {
          correspondence = l_corner;
          best_correspondence = dist_min;
          found = true;
        }
      }

      found &= (best_correspondence < 0.2);
    }

    // - Then, insert the corner into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + l_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      Corner new_corner(new_pt, corner.which_plane_);
      new_corner.n_observations_ = correspondence.n_observations_ + 1;
      grid_map.update(correspondence, new_corner);
    }
    else
    {
      Corner new_corner(l_pt, corner.which_plane_);
      new_corners.push_back(new_corner);

      // Set the occupancy of the 0 altitude map layer using the corner features
      if (grid_map(l_pt.x_, l_pt.y_, 0).data != nullptr)
      {
        if (grid_map(l_pt.x_, l_pt.y_, 0).data->is_occupied_ == nullptr)
        {
          grid_map(l_pt.x_, l_pt.y_, 0).data->is_occupied_ = new bool();
        }
        *grid_map(l_pt.x_, l_pt.y_, 0).data->is_occupied_ = true;
      }
    }
  }

  // Insert the new observations found
  for (const auto& corner : new_corners)
  {
    grid_map.insert(corner);
  }
}

void LidarMapper::globalCornerMap(const Pose& robot_pose, const std::vector<Corner>& corners,
                                  TopologicalMap& topological_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Local array to store the new planar features
  std::vector<Corner> new_corners;

  // ----------------------------------------------------------------------------
  // ------ Insert corner into the grid map
  // ----------------------------------------------------------------------------
  for (auto& corner : corners)
  {
    // - First convert them to map's referential using the robot pose
    Point l_pt = corner.pos_ * tf;

    // - Then, look for correspondences in the local map
    Corner correspondence{};
    float best_correspondence = 0.20;
    bool found = false;

    Cell cell;
    vertex_t node;
    std::vector<Corner> l_corners = topological_map.getCorners(l_pt.x_, l_pt.y_, l_pt.z_, false);

    for (const auto& l_corner : l_corners)
    {
      float dist_min = l_pt.distance(l_corner.pos_);

      if (dist_min < best_correspondence)
      {
        correspondence = l_corner;
        best_correspondence = dist_min;
        found = true;
      }
    }

    found &= (best_correspondence < 0.2);

    // - Then, insert the corner into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + l_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      Corner new_corner(new_pt, corner.which_plane_);
      new_corner.n_observations_ = correspondence.n_observations_ + 1;
      topological_map.update(correspondence, new_corner);
    }
    else
    {
      Corner new_corner(l_pt, corner.which_plane_);
      new_corners.push_back(new_corner);

      // Set the occupancy of the 0 altitude map layer using the corner features
      vertex_t l_node;
      Cell l_cell;
      if (topological_map.getCell(Point(l_pt.x_, l_pt.y_, 0), l_cell, l_node))
      {
        if (l_cell.data != nullptr)
        {
          if (l_cell.data->is_occupied_ == nullptr)
          {
            l_cell.data->is_occupied_ = new bool();
          }
          *l_cell.data->is_occupied_ = true;
        }
      }
    }
  }

  // Insert the new observations found
  for (const auto& corner : new_corners)
  {
    topological_map.insert(corner);
  }
}

void LidarMapper::globalPlanarMap(const Pose& robot_pose, const std::vector<Planar>& planars, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Local array to store the new planar features
  std::vector<Planar> new_planars;

  // ----------------------------------------------------------------------------
  // ------ Insert planar into the grid map
  // ----------------------------------------------------------------------------
  for (auto& planar : planars)
  {
    // - First convert them to map's referential using the robot pose
    Point l_pt = planar.pos_ * tf;

    // - Then, look for correspondences in the local map
    Planar correspondence{};
    float best_correspondence = 0.20;
    bool found = false;
    std::vector<Planar>* l_planars = { nullptr };

    if (grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data != nullptr)
    {
      l_planars = grid_map(l_pt.x_, l_pt.y_, l_pt.z_).data->planar_features_;
    }

    if (l_planars != nullptr)
    {
      for (const auto& l_planar : *l_planars)
      {
        float dist_min = l_pt.distance(l_planar.pos_);

        if (dist_min < best_correspondence)
        {
          correspondence = l_planar;
          best_correspondence = dist_min;
          found = true;
        }
      }

      found &= (best_correspondence < 0.2);
    }

    // - Then, insert the planar into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + l_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      Planar new_planar(new_pt, planar.which_plane_);
      new_planar.n_observations_ = correspondence.n_observations_ + 1;
      grid_map.update(correspondence, new_planar);
    }
    else
    {
      Planar new_planar(l_pt, planar.which_plane_);
      new_planars.push_back(new_planar);
    }
  }

  // Insert the new observations found
  for (const auto& planar : new_planars)
  {
    grid_map.insert(planar);
  }
}

void LidarMapper::globalPlanarMap(const Pose& robot_pose, const std::vector<Planar>& planars,
                                  TopologicalMap& topological_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Local array to store the new planar features
  std::vector<Planar> new_planars;

  // ----------------------------------------------------------------------------
  // ------ Insert corner into the grid map
  // ----------------------------------------------------------------------------
  for (auto& planar : planars)
  {
    // - First convert them to map's referential using the robot pose
    Point l_pt = planar.pos_ * tf;

    // - Then, look for correspondences in the local map
    Planar correspondence{};
    float best_correspondence = 0.20;
    bool found = false;

    std::vector<Planar> l_planars = topological_map.getPlanars(l_pt.x_, l_pt.y_, l_pt.z_, false);

    for (const auto& l_planar : l_planars)
    {
      float dist_min = l_pt.distance(l_planar.pos_);

      if (dist_min < best_correspondence)
      {
        correspondence = l_planar;
        best_correspondence = dist_min;
        found = true;
      }
    }

    found &= (best_correspondence < 0.2);

    // - Then, insert the corner into the grid map
    if (found)
    {
      Point new_pt = ((correspondence.pos_ * static_cast<float>(correspondence.n_observations_)) + l_pt) /
                     static_cast<float>(correspondence.n_observations_ + 1);
      Planar new_planar(new_pt, planar.which_plane_);
      new_planar.n_observations_ = correspondence.n_observations_ + 1;
      topological_map.update(correspondence, new_planar);
    }
    else
    {
      Planar new_planar(l_pt, planar.which_plane_);
      new_planars.push_back(new_planar);
    }
  }

  // Insert the new observations found
  for (const auto& planar : new_planars)
  {
    topological_map.insert(planar);
  }
}

void LidarMapper::globalPlaneMap(const Pose& robot_pose, const std::vector<SemiPlane>& planes, OccupancyMap& grid_map)
{
  // ----------------------------------------------------------------------------
  // ------ First, we remove semiplane outliers in the global map checking their
  // ------        number of correspondences at a well defined frequency
  // ----------------------------------------------------------------------------
  if (it_ % filter_frequency_ == 0)
  {
    int planes_size = static_cast<int>(grid_map.planes_.size());
    for (int i = 0; i < planes_size; i++)
    {
      if (grid_map.planes_[i].n_occurences_ > filter_frequency_ && grid_map.planes_[i].n_correspondences_ < 10)
      {
        grid_map.planes_.erase(grid_map.planes_.begin() + i);
      }
    }
  }

  // ----------------------------------------------------------------------------
  // ------ Search for correspondences between local planes and global planes
  // ------ Two stage process:
  // ------  * (A) Check semi-plane overlap
  // ------  * (B) Compare planes normals
  // ------  *  If (B), then check (C) plane to plane distance
  // ----------------------------------------------------------------------------

  // Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Define correspondence thresholds
  float v_dist = 0.2;   // max vector displacement for all the components
  float sp_dist = 0.3;  // max distance from source plane centroid to target plane
  float area_th = 1.0;  // minimum overlapping area between semiplanes

  // Array to store the new planes observed
  std::vector<SemiPlane> new_planes;

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

    // Declare plane to store the correspondence
    auto* correspondence = new SemiPlane();

    // Convert local plane to maps' referential frame
    SemiPlane l_plane = plane;
    for (auto& point : l_plane.points_)
    {
      point = point * tf;  // Convert plane points
    }
    for (auto& point : l_plane.extremas_)
    {
      point = point * tf;  // Convert plane boundaries
    }
    l_plane.centroid_ = l_plane.centroid_ * tf;                                               // Convert the centroid
    Ransac::estimateNormal(l_plane.points_, l_plane.a_, l_plane.b_, l_plane.c_, l_plane.d_);  // Convert plane normal
    l_plane.setLocalRefFrame();

    bool found = false;
    for (auto& g_plane : grid_map.planes_)
    {
      // Increment the number of visits to the plane (used to filter planes with low correspondences)
      g_plane.n_occurences_++;

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

            // Update number of correspondences
            g_plane.n_correspondences_++;

            // Save the correspondence semi-plane
            correspondence = &g_plane;

            // Set correspondence flag
            found = true;
          }
        }
      }
    }

    // Check if a correspondence was found
    if (found)
    {
      // If so, update plane on global map with new observation (registration)

      // Insert the new observation points into the correspondent global map plane
      correspondence->points_.insert(correspondence->points_.end(), l_plane.points_.begin(), l_plane.points_.end());

      // Re-compute centroid
      correspondence->centroid_ = Point(0, 0, 0);
      for (const auto& pt : correspondence->points_)
      {
        correspondence->centroid_ = correspondence->centroid_ + pt;
      }
      correspondence->centroid_ = correspondence->centroid_ / static_cast<float>(correspondence->points_.size());

      // Re-compute the plane normal
      float l_a, l_b, l_c, l_d;
      Ransac::estimateNormal(correspondence->points_, l_a, l_b, l_c, l_d);
      correspondence->a_ = l_a;
      correspondence->b_ = l_b;
      correspondence->c_ = l_c;
      correspondence->d_ = l_d;
      correspondence->setLocalRefFrame();

      // Re-compute the semi plane boundaries
      Plane filter_plane(l_a, l_b, l_c, l_d, correspondence->points_);
      SemiPlane filter_semiplane;
      QuickConvexHull::process(filter_plane, filter_semiplane);
      correspondence->extremas_ = filter_semiplane.extremas_;
      correspondence->setArea();

      correspondence->points_ =
          correspondence->extremas_;  // This is a trick to improve performance: to update a semiplane, we only need the
                                      // previously calculated extremas and the newly observed points :)
    }
    else
    {
      // If not, add a new plane to the map
      new_planes.push_back(l_plane);
    }
  }

  // Add new planes found to the map
  if (!new_planes.empty())
  {
    grid_map.planes_.insert(grid_map.planes_.end(), new_planes.begin(), new_planes.end());
  }
}

void LidarMapper::globalPlaneMap(const Pose& robot_pose, const std::vector<SemiPlane>& planes,
                                 TopologicalMap& topological_map)
{
  // ----------------------------------------------------------------------------
  // ------ First, we remove semiplane outliers in the global map checking their
  // ------        number of correspondences at a well defined frequency
  // ----------------------------------------------------------------------------
  if (it_ % filter_frequency_ == 0)
  {
    int planes_size = static_cast<int>(topological_map.planes_.size());
    for (int i = 0; i < planes_size; i++)
    {
      if (topological_map.planes_[i].n_occurences_ > filter_frequency_ &&
          topological_map.planes_[i].n_correspondences_ < 10)
      {
        topological_map.planes_.erase(topological_map.planes_.begin() + i);
      }
    }
  }

  // ----------------------------------------------------------------------------
  // ------ Search for correspondences between local planes and global planes
  // ------ Two stage process:
  // ------  * (A) Check semi-plane overlap
  // ------  * (B) Compare planes normals
  // ------  *  If (B), then check (C) plane to plane distance
  // ----------------------------------------------------------------------------

  // Convert robot pose into homogeneous transformation
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // Define correspondence thresholds
  float v_dist = 0.2;   // max vector displacement for all the components
  float sp_dist = 0.3;  // max distance from source plane centroid to target plane
  float area_th = 1.0;  // minimum overlapping area between semiplanes

  // Array to store the new planes observed
  std::vector<SemiPlane> new_planes;

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

    // Declare plane to store the correspondence
    auto* correspondence = new SemiPlane();

    // Convert local plane to maps' referential frame
    SemiPlane l_plane = plane;
    for (auto& point : l_plane.points_)
    {
      point = point * tf;  // Convert plane points
    }
    for (auto& point : l_plane.extremas_)
    {
      point = point * tf;  // Convert plane boundaries
    }
    l_plane.centroid_ = l_plane.centroid_ * tf;                                               // Convert the centroid
    Ransac::estimateNormal(l_plane.points_, l_plane.a_, l_plane.b_, l_plane.c_, l_plane.d_);  // Convert plane normal
    l_plane.setLocalRefFrame();

    bool found = false;
    for (auto& g_plane : topological_map.planes_)
    {
      // Increment the number of visits to the plane (used to filter planes with low correspondences)
      g_plane.n_occurences_++;

      if (g_plane.points_.empty())
      {
        continue;
      }

      // --------------------------------
      // (A) - Check semi-plane overlap
      // --------------------------------

      // First project the global and local plane extremas to the global plane reference frame
      Tf ref_frame = g_plane.local_ref_.inverse();
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

            // Update number of correspondences
            g_plane.n_correspondences_++;

            // Save the correspondence semi-plane
            correspondence = &g_plane;

            // Set correspondence flag
            found = true;
          }
        }
      }
    }

    // Check if a correspondence was found
    if (found)
    {
      // If so, update plane on global map with new observation (registration)

      // Insert the new observation points into the correspondent global map plane
      correspondence->points_.insert(correspondence->points_.end(), l_plane.points_.begin(), l_plane.points_.end());

      // Re-compute centroid
      correspondence->centroid_ = Point(0, 0, 0);
      for (const auto& pt : correspondence->points_)
      {
        correspondence->centroid_ = correspondence->centroid_ + pt;
      }
      correspondence->centroid_ = correspondence->centroid_ / static_cast<float>(correspondence->points_.size());

      // Re-compute the plane normal
      float l_a, l_b, l_c, l_d;
      Ransac::estimateNormal(correspondence->points_, l_a, l_b, l_c, l_d);
      correspondence->a_ = l_a;
      correspondence->b_ = l_b;
      correspondence->c_ = l_c;
      correspondence->d_ = l_d;
      correspondence->setLocalRefFrame();

      // Re-compute the semi plane boundaries
      Plane filter_plane(l_a, l_b, l_c, l_d, correspondence->points_);
      SemiPlane filter_semiplane;
      QuickConvexHull::process(filter_plane, filter_semiplane);
      correspondence->extremas_ = filter_semiplane.extremas_;
      correspondence->setArea();

      correspondence->points_ =
          correspondence->extremas_;  // This is a trick to improve performance: to update a semiplane, we only need the
                                      // previously calculated extremas and the newly observed points :)
    }
    else
    {
      // If not, add a new plane to the map
      new_planes.push_back(l_plane);
    }
  }

  // Add new planes found to the map
  if (!new_planes.empty())
  {
    topological_map.planes_.insert(topological_map.planes_.end(), new_planes.begin(), new_planes.end());
  }
}

void LidarMapper::globalElevationMap(const Pose& robot_pose, const Plane& ground_plane, ElevationMap& elevation_map)
{
  // ----------------------------------------------------------------------------
  // ------ Convert robot pose into homogeneous transformation
  // ----------------------------------------------------------------------------
  std::array<float, 9> Rot{};
  robot_pose.toRotMatrix(Rot);
  std::array<float, 3> trans = { robot_pose.x_, robot_pose.y_, robot_pose.z_ };
  Tf tf(Rot, trans);

  // ----------------------------------------------------------------------------
  // ------ Add new altemetry measures from ground plane to the elevation map
  // ----------------------------------------------------------------------------
  for (const auto& pt : ground_plane.points_)
  {
    Point l_pt = pt * tf;
    elevation_map.update(l_pt.z_, l_pt.x_, l_pt.y_);
  }

  // ----------------------------------------------------------------------------
  // ------ Add new altemetry measures from robot pose to the elevation map
  // ----------------------------------------------------------------------------

  // Discretize robot box using the elevation map resolution
  for (float i = 0; i < robot_dim_x_;)
  {
    for (float j = 0; j < robot_dim_y_;)
    {
      Point pt(i - robot_dim_x_ / 2, j - robot_dim_y_ / 2, 0);
      Point pt_transformed = pt * tf;

      elevation_map.update(pt_transformed.z_, pt_transformed.x_, pt_transformed.y_);

      j += elevation_map.resolution_;
    }
    i += elevation_map.resolution_;
  }
}

}  // namespace vineslam
