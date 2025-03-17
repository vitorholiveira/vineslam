#include <vineslam/mapping/lidar_mapping_omnidirectional.hpp>

namespace vineslam
{
OmnidirectionalMapper::OmnidirectionalMapper(const Parameters& params, const std::string& lidar_type)
{
  if (lidar_type == "Velodyne")
  {
    // Set velodyne configuration parameters
    picked_num_ = 2;
    planes_th_ = static_cast<float>(60.) * DEGREE_TO_RAD;
    ground_th_ = static_cast<float>(3.) * DEGREE_TO_RAD;
    edge_threshold_ = 0.1;
    planar_threshold_ = 0.1;
    vertical_scans_ = 16;
    horizontal_scans_ = 1800;
    ground_scan_idx_ = 7;
    segment_valid_point_num_ = 5;
    segment_valid_line_num_ = 3;
    vertical_angle_bottom_ = static_cast<float>(15. + 0.1) * DEGREE_TO_RAD;
    ang_res_x_ = static_cast<float>(0.2) * DEGREE_TO_RAD;
    ang_res_y_ = static_cast<float>(2.) * DEGREE_TO_RAD;
  }
  else if (lidar_type == "Robosense")
  {
    // Set robosense configuration parameters
    picked_num_ = 2;
    planes_th_ = static_cast<float>(40.) * DEGREE_TO_RAD;
    ground_th_ = static_cast<float>(3.) * DEGREE_TO_RAD;
    edge_threshold_ = 0.1;
    planar_threshold_ = 0.1;
    vertical_scans_ = 32;
    horizontal_scans_ = 1800;
    ground_scan_idx_ = 20;
    segment_valid_point_num_ = 5;
    segment_valid_line_num_ = 3;
    vertical_angle_bottom_ = static_cast<float>(55.0) * DEGREE_TO_RAD;
    ang_res_x_ = static_cast<float>(360.0 / horizontal_scans_) * DEGREE_TO_RAD;
    ang_res_y_ = static_cast<float>(70.0 / (vertical_scans_ - 1)) * DEGREE_TO_RAD;
  }
  else if (lidar_type == "Ouster")
  {
    // Set ouster configuration parameters
    picked_num_ = 2;
    planes_th_ = static_cast<float>(40.) * DEGREE_TO_RAD;
    ground_th_ = static_cast<float>(3.) * DEGREE_TO_RAD;
    edge_threshold_ = 0.1;
    planar_threshold_ = 0.1;
    vertical_scans_ = 64;
    horizontal_scans_ = 2048;
    ground_scan_idx_ = 15;
    segment_valid_point_num_ = 5;
    segment_valid_line_num_ = 3;
    vertical_angle_bottom_ = static_cast<float>(21.1) * DEGREE_TO_RAD;
    ang_res_x_ = static_cast<float>(360.0 / horizontal_scans_) * DEGREE_TO_RAD;
    ang_res_y_ = static_cast<float>(42.37 / (vertical_scans_ - 1)) * DEGREE_TO_RAD;
  }

  // Set robot dimensions for elevation map computation
  robot_dim_x_ = params.robot_dim_x_;
  robot_dim_y_ = params.robot_dim_y_;
  robot_dim_z_ = params.robot_dim_z_;

  // Set previous robot pose handler
  prev_robot_pose_ = Pose(0, 0, 0, 0, 0, 0);

  // Set parameters object handler
  params_ = params;

  // Initialize thread pool
  thread_pool_ = new lama::ThreadPool;
  thread_pool_->init(NUM_THREADS);
}

void OmnidirectionalMapper::flatGroundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
{
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (int j = 0; j < horizontal_scans_; j++)
  {
    for (int i = 0; i < ground_scan_idx_; i++)
    {
      int lower_idx = j + i * horizontal_scans_;
      int upper_idx = j + (i + 1) * horizontal_scans_;

      Point upper_pt = in_pts[upper_idx];
      Point lower_pt = in_pts[lower_idx];

      if (range_mat_(i, j) == -1 || range_mat_(i + 1, j) == -1)
      {
        // no info to check, invalid points
        //        ground_mat_(i, j) = -1;
        continue;
      }

      float dX = upper_pt.x_ - lower_pt.x_;
      float dY = upper_pt.y_ - lower_pt.y_;
      float dZ = upper_pt.z_ - lower_pt.z_;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th_ && std::fabs(lower_pt.z_) > (-laser2base_z_ / 2) &&
          std::fabs(upper_pt.z_) > (-laser2base_z_ / 2))
      {
        out_pcl.points_.push_back(lower_pt);
        out_pcl.points_.push_back(upper_pt);
        out_pcl.indexes_.emplace_back(i, j);
        out_pcl.indexes_.emplace_back(i + 1, j);
      }
    }
  }
}

void OmnidirectionalMapper::groundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
{
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (int j = 0; j < horizontal_scans_; j++)
  {
    for (int i = 0; i < ground_scan_idx_; i++)
    {
      int lower_idx = j + i * horizontal_scans_;
      int upper_idx = j + (i + 1) * horizontal_scans_;

      Point upper_pt = in_pts[upper_idx];
      Point lower_pt = in_pts[lower_idx];

      if (range_mat_(i, j) == -1 || range_mat_(i + 1, j) == -1)
      {
        // no info to check, invalid points
        //        ground_mat_(i, j) = -1;
        continue;
      }

      float dX = upper_pt.x_ - lower_pt.x_;
      float dY = upper_pt.y_ - lower_pt.y_;
      float dZ = upper_pt.z_ - lower_pt.z_;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th_)
      {
        out_pcl.points_.push_back(lower_pt);
        out_pcl.points_.push_back(upper_pt);
        out_pcl.indexes_.emplace_back(i, j);
        out_pcl.indexes_.emplace_back(i + 1, j);
      }
    }
  }
}

void OmnidirectionalMapper::cloudSegmentation(const std::vector<Point>& in_pts, std::vector<PlanePoint>& cloud_seg)
{
  // Segmentation process
  int label = 1;
  for (int i = 0; i < vertical_scans_; i++)
  {
#if NUM_THREADS > 1
    thread_pool_->enqueue([this, i, &label]() {
#endif
      for (int j = 0; j < horizontal_scans_; j++)
      {
        if (label_mat_(i, j) == 0 && range_mat_(i, j) != -1)
          labelComponents(i, j, label);
      }
#if NUM_THREADS > 1
    });
#endif
  }
#if NUM_THREADS > 1
  thread_pool_->wait();
#endif

  // Extract segmented cloud for visualization
  int seg_cloud_size = 0;
  for (int i = 0; i < vertical_scans_; i++)
  {
    seg_pcl_.start_col_idx[i] = seg_cloud_size - 1 + 5;

    for (int j = 0; j < horizontal_scans_; j++)
    {
      if (label_mat_(i, j) > 0 || ground_mat_(i, j) == 1)
      {
        if (label_mat_(i, j) == 999999)
          continue;

        // The majority of ground points are skipped
        if (ground_mat_(i, j) == 1)
        {
          //          if (j % 20 != 0 && j > 20 && j < horizontal_scans_ - 20)
          if (j % 5 != 0 && j > 5 && j < horizontal_scans_ - 5)
          {
            continue;
          }
        }

        // Mark ground points so they will not be considered as edge features later
        seg_pcl_.is_ground[seg_cloud_size] = (ground_mat_(i, j) == 1);

        // Save segmented cloud into a pcl
        Point pt = in_pts[j + i * horizontal_scans_];
        PlanePoint plane_pt(pt, label_mat_(i, j));
        cloud_seg.push_back(plane_pt);
        // ------------------------------------------
        // Save segmented cloud in the given structure
        seg_pcl_.col_idx[seg_cloud_size] = j;
        seg_pcl_.range[seg_cloud_size] = range_mat_(i, j);
        seg_cloud_size++;
        // ------------------------------------------
      }
    }
    seg_pcl_.end_col_idx[i] = seg_cloud_size - 1 - 5;
  }
}

void OmnidirectionalMapper::labelComponents(const int& row, const int& col, int& label)
{
  using Coord2D = Eigen::Vector2i;
  std::deque<Coord2D> queue;
  std::deque<Coord2D> global_queue;

  queue.emplace_back(row, col);
  global_queue.emplace_back(row, col);

  std::vector<bool> line_count_flag(vertical_scans_, false);

  // - Define neighborhood
  const Coord2D neighbor_it[4] = { { 0, -1 }, { -1, 0 }, { 1, 0 }, { 0, 1 } };

  while (!queue.empty())
  {
    // Evaluate front element of the queue and pop it
    Coord2D from_idx = queue.front();
    queue.pop_front();

    // Mark popped point as belonging to the segment
    label_mat_(from_idx.x(), from_idx.y()) = label;

    // Compute point from range image
    float d1 = range_mat_(from_idx.x(), from_idx.y());

    // Loop through all the neighboring grids of popped grid
    for (const auto& iter : neighbor_it)
    {
      // Compute new index
      int c_idx_x = from_idx.x() + iter.x();
      int c_idx_y = from_idx.y() + iter.y();

      // Check if index is within the boundary
      if (c_idx_x < 0 || c_idx_x >= vertical_scans_)
        continue;
      if (c_idx_y < 0)
        c_idx_y = horizontal_scans_ - 1;
      if (c_idx_y >= horizontal_scans_)
        c_idx_y = 0;

      // Prevent infinite loop (caused by put already examined point back)
      if (label_mat_(c_idx_x, c_idx_y) != 0)
        continue;

      // Compute point from range image
      float d2 = range_mat_(c_idx_x, c_idx_y);

      float dmax = std::max(d1, d2);
      float dmin = std::min(d1, d2);

      // Compute angle between the two points
      float alpha = (iter.x() == 0) ? ang_res_x_ : ang_res_y_;

      // Compute beta and check if points belong to the same segment
      auto beta = std::atan2((dmin * std::sin(alpha)), (dmax - dmin * std::cos(alpha)));
      if (beta > planes_th_)
      {
        queue.emplace_back(c_idx_x, c_idx_y);
        global_queue.emplace_back(c_idx_x, c_idx_y);

        label_mat_(c_idx_x, c_idx_y) = label;
        line_count_flag[c_idx_x] = true;
      }
    }
  }

  // Check if this segment is valid
  bool feasible_segment = false;
  if (global_queue.size() >= 30)
  {
    feasible_segment = true;
  }
  else if (static_cast<int>(global_queue.size()) >= segment_valid_point_num_)
  {
    int line_count = 0;
    for (int i = 0; i < vertical_scans_; i++)
    {
      if (line_count_flag[i])
        line_count++;
    }

    if (line_count >= segment_valid_line_num_)
      feasible_segment = true;
  }

  if (feasible_segment)
  {
    label++;
  }
  else
  {
    for (auto& i : global_queue)
      label_mat_(i.x(), i.y()) = 999999;
  }
}

void OmnidirectionalMapper::extractHighLevelPlanes(const std::vector<Point>& in_pts, const SemiPlane& ground_plane,
                                                   std::vector<SemiPlane>& out_planes)
{
  Tf tf;
  std::array<float, 9> tf_rot{};
  prev_robot_pose_.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ 0, 0, 0 });

  // Remove ground and null points from the set of input points
  std::vector<Point> non_ground{};
  for (const auto& pt : in_pts)
  {
    if (ground_plane.point2Plane(pt) > 0.2 && pt != Point(0, 0, 0))
    {
      non_ground.push_back(pt);
    }
  }

  // -------------------------------------------------------------------------------
  // ----- Segment plane points in two different sets
  // -------------------------------------------------------------------------------
  // - Start by computing the average of the y component of all points
  float y_mean = 0.;
  for (auto const& plane_pt : non_ground)
    y_mean += plane_pt.y_;
  y_mean /= static_cast<float>(non_ground.size());

  // - Cluster points using the y mean threshold
  Plane side_plane_a, side_plane_b;
  for (auto const& plane_pt : non_ground)
  {
    Point rotated_to_map_pt = plane_pt * tf;
    PlanePoint m_plane_pt;

    m_plane_pt.pos_ = plane_pt;
    m_plane_pt.which_plane_ = (rotated_to_map_pt.y_ < y_mean) ? 0 : 1;

    if (m_plane_pt.which_plane_ == 0)
    {
      side_plane_a.points_.push_back(m_plane_pt.pos_);
    }
    else
    {
      side_plane_b.points_.push_back(m_plane_pt.pos_);
    }
  }

  // - Remove outliers using RANSAC
#if VERBOSE == 1
  t_->tick("lidar_mapper::highLevelPlanes::ransac(1)");
#endif
  std::vector<Plane> planes = {};
  Plane side_plane_a_filtered, side_plane_b_filtered;
  if (Ransac::process(side_plane_a.points_, side_plane_a_filtered, 128, 0.10, true) &&
      side_plane_a_filtered.points_.size() < 7000 &&
      side_plane_a_filtered.points_.size() > 10)  // prevent dense planes and slow convex hulls
  {
    side_plane_a_filtered.id_ = 0;
    planes.push_back(side_plane_a_filtered);
  }
#if VERBOSE == 1
  t_->tock();
#endif
#if VERBOSE == 1
  t_->tick("lidar_mapper::highLevelPlanes::ransac(2)");
#endif
  if (Ransac::process(side_plane_b.points_, side_plane_b_filtered, 128, 0.10, true) &&
      side_plane_b_filtered.points_.size() < 7000 &&
      side_plane_b_filtered.points_.size() > 10)  // prevent dense planes and slow convex hulls
  {
    side_plane_b_filtered.id_ = 1;
    planes.push_back(side_plane_b_filtered);
  }
#if VERBOSE == 1
  t_->tock();
#endif

  // -------------------------------------------------------------------------------
  // ----- Check the validity of the extracted planes
  // -------------------------------------------------------------------------------
  for (auto& plane : planes)
  {
    // Check if the plane have a minimum number of points
    plane.centroid_ = Point(0, 0, 0);
    for (const auto& pt : plane.points_)
    {
      plane.centroid_ = plane.centroid_ + pt;
    }
    plane.centroid_ = plane.centroid_ / static_cast<float>(plane.points_.size());
    plane.setLocalRefFrame();

    SemiPlane l_semi_plane;
#if VERBOSE == 1
    t_->tick("lidar_mapper::highLevelPlanes::convexHull(...)");
#endif
    bool ch = QuickConvexHull::process(plane, l_semi_plane);
    if (ch && checkPlaneConsistency(l_semi_plane, ground_plane))
    {
      out_planes.push_back(l_semi_plane);
    }
#if VERBOSE == 1
    t_->tock();
#endif
  }
}

bool OmnidirectionalMapper::checkPlaneConsistency(const SemiPlane& plane, const SemiPlane& ground_plane)
{
  // A - Check semiplane area
  if (plane.area_ < 4)
  {
    return false;
  }

  // B - Check if the points belonging to the semiplane are continuous
  //  Point p0;
  //  if (plane.points_.empty())
  //  {
  //    return false;
  //  }
  //  else
  //  {
  //    p0 = plane.points_[0];
  //  }
  //  std::vector<Point> pts = plane.points_;
  //  float d0 = 0;
  //  while (pts.size() >= 2)  // Find nearest neighbor of p0, pop it from the vector, compare distances computed
  //                           // between iterations, find holes by large variations on the distance measured
  //  {
  //    float d1, min_dist = std::numeric_limits<float>::max();
  //    uint32_t idx = 0;
  //    for (uint32_t i = 0; i < pts.size(); ++i)
  //    {
  //      d1 = p0.distance(pts[i]);
  //      if (d1 != 0 && d1 < min_dist)
  //      {
  //        min_dist = d1;
  //        idx = i;
  //      }
  //    }
  //    d1 = min_dist;pts.begin() + idx);
  //      d0 = d1;
  //    }
  //    if (std::fabs(d1 - d0) > 0.2 && d0 != 0)  // We found a hole in this case ...
  //    {
  //      return false;
  //    }
  //    else
  //    {
  //      pts.erase(
  //  }

  // C - Make sure that the plane is not horizontal
  float dot = Vec(plane.a_, plane.b_, plane.c_).dot(Vec(ground_plane.a_, ground_plane.b_, ground_plane.c_));
  if (std::fabs(dot) > 1.0)
  {
    return false;
  }

  return true;
}

void OmnidirectionalMapper::extractFeatures(const std::vector<PlanePoint>& in_plane_pts,
                                            std::vector<Corner>& out_corners, std::vector<Planar>& out_planars)
{
  // -------------------------------------------------------------------------------
  // ----- Compute cloud smoothness
  // -------------------------------------------------------------------------------
  int* cloudPlanarLabel = new int[vertical_scans_ * horizontal_scans_];
  int* cloudCornerLabel = new int[vertical_scans_ * horizontal_scans_];
  int l_cloud_size = in_plane_pts.size();
  std::vector<smoothness_t> cloud_smoothness(vertical_scans_ * horizontal_scans_);
  std::vector<int> neighbor_picked(vertical_scans_ * horizontal_scans_);
  for (int i = 5; i < l_cloud_size - 5; i++)
  {
    // Compute smoothness and save it
    float diff_range = seg_pcl_.range[i - 5] + seg_pcl_.range[i - 4] + seg_pcl_.range[i - 3] + seg_pcl_.range[i - 2] +
                       seg_pcl_.range[i - 1] + seg_pcl_.range[i + 1] + seg_pcl_.range[i + 2] + seg_pcl_.range[i + 3] +
                       seg_pcl_.range[i + 4] + seg_pcl_.range[i + 5] - 10 * seg_pcl_.range[i];

    cloud_smoothness[i].value = diff_range * diff_range;
    cloud_smoothness[i].idx = i;

    cloudPlanarLabel[i] = 0;
    cloudCornerLabel[i] = 0;

    // Reset neighborhood flag array
    neighbor_picked[i] = 0;
  }

  // -------------------------------------------------------------------------------
  // ----- Mark occluded points
  // -------------------------------------------------------------------------------

  for (int i = 5; i < l_cloud_size - 6; ++i)
  {
    float depth1 = seg_pcl_.range[i];
    float depth2 = seg_pcl_.range[i + 1];
    int col_diff = std::abs(int(seg_pcl_.col_idx[i + 1] - seg_pcl_.col_idx[i]));

    if (col_diff < 10)
    {
      if (depth1 - depth2 > 0.3)
      {
        neighbor_picked[i - 5] = 1;
        neighbor_picked[i - 4] = 1;
        neighbor_picked[i - 3] = 1;
        neighbor_picked[i - 2] = 1;
        neighbor_picked[i - 1] = 1;
        neighbor_picked[i] = 1;
      }
      else if (depth2 - depth1 > 0.3)
      {
        neighbor_picked[i + 1] = 1;
        neighbor_picked[i + 2] = 1;
        neighbor_picked[i + 3] = 1;
        neighbor_picked[i + 4] = 1;
        neighbor_picked[i + 5] = 1;
        neighbor_picked[i + 6] = 1;
      }
    }

    float diff1 = std::abs(float(seg_pcl_.range[i - 1] - seg_pcl_.range[i]));
    float diff2 = std::abs(float(seg_pcl_.range[i + 1] - seg_pcl_.range[i]));

    if (diff1 > 0.02 * seg_pcl_.range[i] && diff2 > 0.02 * seg_pcl_.range[i])
      neighbor_picked[i] = 1;
  }

  // -------------------------------------------------------------------------------
  // ----- Extract features from the 3D cloud
  // -------------------------------------------------------------------------------
  std::vector<Planar> planar_points_less_flat;
  int corner_id = 0;
  int planar_id = 0;
  for (int i = 0; i < vertical_scans_; i++)
  {
    planar_points_less_flat.clear();

    for (int k = 0; k < 6; k++)
    {
      // Compute start and end indexes of the sub-region
      int sp = (seg_pcl_.start_col_idx[i] * (6 - k) + (seg_pcl_.end_col_idx[i] * k)) / 6;
      int ep = (seg_pcl_.start_col_idx[i] * (5 - k) + (seg_pcl_.end_col_idx[i] * (k + 1))) / 6 - 1;

      if (sp >= ep)
        continue;

      // Sort smoothness values for the current sub-region
      std::sort(cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep, by_value());

      // -- Extract edge features
      int picked_counter = 0;
      for (int l = ep; l >= sp; l--)
      {
        int idx = cloud_smoothness[l].idx;

        // Check if the current point is an edge feature
        if (neighbor_picked[idx] == 0 && cloud_smoothness[l].value > edge_threshold_ && !seg_pcl_.is_ground[idx])
        {
          picked_counter++;
          if (picked_counter <= picked_num_)
          {
            Corner l_corner(in_plane_pts[idx].pos_, in_plane_pts[idx].which_plane_, corner_id);
            out_corners.push_back(l_corner);
            corner_id++;
          }
          else
          {
            break;
          }

          cloudCornerLabel[idx] = -1;

          // Mark neighbor points to reject as future features
          neighbor_picked[idx] = 1;
          for (int m = 1; m <= 5; m++)
          {
            if (idx + m >= static_cast<int>(seg_pcl_.col_idx.size()))
              continue;
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m - 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
          for (int m = -1; m >= -5; m--)
          {
            if (idx + m < 0)
              continue;
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m + 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
        }
      }

      // -- Extract planar features
      picked_counter = 0;
      for (int l = sp; l <= ep; l++)
      {
        int idx = cloud_smoothness[l].idx;

        // Check if the current point is a planar feature
        if (neighbor_picked[idx] == 0 && cloud_smoothness[l].value < planar_threshold_)
        {
          cloudPlanarLabel[idx] = -1;

          picked_counter++;
          if (picked_counter >= 4)
            break;

          neighbor_picked[idx] = 1;
          for (int m = 1; m <= 5; m++)
          {
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m - 1]);

            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
          for (int m = -1; m >= -5; m--)
          {
            if (idx + m < 0)
              continue;
            int col_diff = std::abs(seg_pcl_.col_idx[idx + m] - seg_pcl_.col_idx[idx + m + 1]);
            if (col_diff > 10)
              break;
            else
              neighbor_picked[idx + m] = 1;
          }
        }
      }

      for (int l = sp; l <= ep; l++)
      {
        if (cloudPlanarLabel[l] <= 0 && cloudCornerLabel[l] >= 0)
        {
          Planar planar(in_plane_pts[l].pos_, in_plane_pts[l].which_plane_, planar_id);
          planar_points_less_flat.push_back(planar);
        }
      }
    }

    out_planars.insert(out_planars.end(), planar_points_less_flat.begin(), planar_points_less_flat.end());
  }

  free(cloudCornerLabel);
  free(cloudPlanarLabel);
}

void OmnidirectionalMapper::reset()
{
  range_mat_.resize(vertical_scans_, horizontal_scans_);
  ground_mat_.resize(vertical_scans_, horizontal_scans_);
  label_mat_.resize(vertical_scans_, horizontal_scans_);
  range_mat_.fill(-1);
  ground_mat_.setZero();
  label_mat_.setZero();

  int cloud_size = vertical_scans_ * horizontal_scans_;

  seg_pcl_.start_col_idx.assign(vertical_scans_, 0);
  seg_pcl_.end_col_idx.assign(vertical_scans_, 0);
  seg_pcl_.is_ground.assign(cloud_size, false);
  seg_pcl_.col_idx.assign(cloud_size, 0);
  seg_pcl_.range.assign(cloud_size, 0);
}

void OmnidirectionalMapper::localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners,
                                     std::vector<Planar>& out_planars, std::vector<SemiPlane>& out_planes,
                                     SemiPlane& out_groundplane)
{
  // Build velodyne to base_link transformation matrix
  Pose tf_pose(laser2base_x_, laser2base_y_, laser2base_z_, laser2base_roll_, laser2base_pitch_, laser2base_yaw_);
  Tf tf;
  std::array<float, 9> tf_rot{};
  tf_pose.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ tf_pose.x_, tf_pose.y_, tf_pose.z_ }).inverse();

  // Reset global variables and members
  reset();

  // Range image projection
  const size_t cloud_size = pcl.size();
  std::vector<Point> transformed_pcl(vertical_scans_ * horizontal_scans_);
  for (size_t i = 0; i < cloud_size; ++i)
  {
    Point l_pt = pcl[i];

    float range = l_pt.norm3D();

    // find the row and column index in the image for this point
    float vertical_angle = std::atan2(l_pt.z_, std::sqrt(l_pt.x_ * l_pt.x_ + l_pt.y_ * l_pt.y_));

    int row_idx = static_cast<int>((vertical_angle + vertical_angle_bottom_) / ang_res_y_);
    ang_res_y_ = static_cast<float>(2.) * DEGREE_TO_RAD;
    if (row_idx < 0 || row_idx >= vertical_scans_)
    {
      continue;
    }

    float horizon_angle = std::atan2(l_pt.x_, l_pt.y_);  // this is not an error

    int column_idx = static_cast<int>(-round((horizon_angle - M_PI_2) / ang_res_x_) + horizontal_scans_ / 2.);

    if (column_idx >= horizontal_scans_)
    {
      column_idx -= horizontal_scans_;
    }

    if (column_idx < 0 || column_idx >= horizontal_scans_)
    {
      continue;
    }

    if (range > 50.0 || (std::fabs(l_pt.x_) < robot_dim_x_ && std::fabs(l_pt.y_) < robot_dim_y_))
    {
      continue;
    }

    range_mat_(row_idx, column_idx) = range;

    size_t idx = column_idx + row_idx * horizontal_scans_;
    transformed_pcl[idx] = l_pt;
  }

  // Ground plane processing
  Plane unfiltered_gplane, filtered_gplane;
  // A - Extraction
#if VERBOSE == 1
  t_->tick("lidarMapper::flatGroundRemoval()");
#endif
  flatGroundRemoval(transformed_pcl, unfiltered_gplane);
#if VERBOSE == 1
  t_->tock();
#endif
  // B - Filtering
#if VERBOSE == 1
  t_->tick("lidarMapper::ransac()");
#endif
  Ransac::process(unfiltered_gplane.points_, filtered_gplane, 100, 0.01, true);
#if VERBOSE == 1
  t_->tock();
#endif
  // C - Centroid calculation
  for (const auto& pt : filtered_gplane.points_)
  {
    filtered_gplane.centroid_ = filtered_gplane.centroid_ + pt;
  }
  filtered_gplane.centroid_ = filtered_gplane.centroid_ / static_cast<float>(filtered_gplane.points_.size());
  // D - Bounding polygon
#if VERBOSE == 1
  t_->tick("lidarMapper::convexHull()");
#endif
  QuickConvexHull::process(filtered_gplane, out_groundplane);
#if VERBOSE == 1
  t_->tock();
#endif
  for (auto& pt : out_groundplane.points_)
  {
    pt = pt * tf;
  }
  for (auto& pt : out_groundplane.extremas_)
  {
    pt = pt * tf;
  }
  out_groundplane.centroid_ = out_groundplane.centroid_ * tf;
  // E - Check plane consistency (set to null if not consistent)
  if (out_groundplane.area_ < 4.)
  {
    out_groundplane = SemiPlane();
  }

  // -------------------------------------------------------------------------------
  // ----- Mark raw ground points
  // -------------------------------------------------------------------------------
  Plane non_flat_ground;
#if VERBOSE == 1
  t_->tick("lidarMapper::groundRemoval()");
#endif
  groundRemoval(transformed_pcl, non_flat_ground);
#if VERBOSE == 1
  t_->tock();
#endif
  for (const auto& index : non_flat_ground.indexes_)
  {
    int i = static_cast<int>(index.x_);
    int j = static_cast<int>(index.y_);

    ground_mat_(i, j) = 1;
    label_mat_(i, j) = -1;
  }

  // Planes that are not the ground
  std::vector<PlanePoint> cloud_seg;
#if VERBOSE == 1
  t_->tick("lidarMapper::cloudSegmentation()");
#endif
  cloudSegmentation(transformed_pcl, cloud_seg);
#if VERBOSE == 1
  t_->tock();
#endif

  // Corners feature extraction
#if VERBOSE == 1
  t_->tick("lidarMapper::extractFeatures()");
#endif
  extractFeatures(cloud_seg, out_corners, out_planars);
#if VERBOSE == 1
  t_->tock();
#endif

  if (params_.use_vertical_planes_)
  {
    // Extract high level planes, and then convert them to semi-planes
    // NOTE: Here we can consider all the LiDAR points, the planar features, or the corner features.
    //       At this moment we are considering the corner features because they are less, so the algorithm runs faster
    std::vector<Point> points_to_consider;
    for (auto p : out_planars)
      points_to_consider.push_back(p.pos_);
    extractHighLevelPlanes(points_to_consider, out_groundplane, out_planes);
  }

  // Convert local maps to base link
  for (auto& corner : out_corners)
  {
    corner.pos_ = corner.pos_ * tf;
  }
  for (auto& planar : out_planars)
  {
    planar.pos_ = planar.pos_ * tf;
  }
  for (auto& plane : out_planes)
  {
    for (auto& pt : plane.points_)
    {
      pt = pt * tf;
    }
    for (auto& pt : plane.extremas_)
    {
      pt = pt * tf;
    }
    plane.centroid_ = plane.centroid_ * tf;
  }
}

void OmnidirectionalMapper::localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners,
                                     std::vector<Planar>& out_planars, SemiPlane& out_groundplane)
{
  // Build velodyne to base_link transformation matrix
  Pose tf_pose(laser2base_x_, laser2base_y_, laser2base_z_, laser2base_roll_, laser2base_pitch_, laser2base_yaw_);
  Tf tf;
  std::array<float, 9> tf_rot{};
  tf_pose.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ tf_pose.x_, tf_pose.y_, tf_pose.z_ }).inverse();

  // Reset global variables and members
  reset();

  // Range image projection
  const size_t cloud_size = pcl.size();
  std::vector<Point> transformed_pcl(vertical_scans_ * horizontal_scans_);
  for (size_t i = 0; i < cloud_size; ++i)
  {
    Point l_pt = pcl[i];

    float range = l_pt.norm3D();

    // find the row and column index in the image for this point
    float vertical_angle = std::atan2(l_pt.z_, std::sqrt(l_pt.x_ * l_pt.x_ + l_pt.y_ * l_pt.y_));

    int row_idx = static_cast<int>((vertical_angle + vertical_angle_bottom_) / ang_res_y_);
    ang_res_y_ = static_cast<float>(2.) * DEGREE_TO_RAD;
    if (row_idx < 0 || row_idx >= vertical_scans_)
    {
      continue;
    }

    float horizon_angle = std::atan2(l_pt.x_, l_pt.y_);  // this is not an error

    int column_idx = static_cast<int>(-round((horizon_angle - M_PI_2) / ang_res_x_) + horizontal_scans_ / 2.);

    if (column_idx >= horizontal_scans_)
    {
      column_idx -= horizontal_scans_;
    }

    if (column_idx < 0 || column_idx >= horizontal_scans_)
    {
      continue;
    }

    if (range > 50.0 || (std::fabs(l_pt.x_) < robot_dim_x_ && std::fabs(l_pt.y_) < robot_dim_y_))
    {
      continue;
    }

    range_mat_(row_idx, column_idx) = range;

    size_t idx = column_idx + row_idx * horizontal_scans_;
    transformed_pcl[idx] = l_pt;
  }

  // - Ground plane processing
  Plane unfiltered_gplane, filtered_gplane;
  // A - Extraction
#if VERBOSE == 1
  t_->tick("lidarMapper::flatGroundRemoval()");
#endif
  flatGroundRemoval(transformed_pcl, unfiltered_gplane);
#if VERBOSE == 1
  t_->tock();
#endif
  // B - Filtering
#if VERBOSE == 1
  t_->tick("lidarMapper::ransac()");
#endif
  Ransac::process(unfiltered_gplane.points_, filtered_gplane, 100, 0.01, true);
#if VERBOSE == 1
  t_->tock();
#endif
  // C - Centroid calculation
  for (const auto& pt : filtered_gplane.points_)
  {
    filtered_gplane.centroid_ = filtered_gplane.centroid_ + pt;
  }
  filtered_gplane.centroid_ = filtered_gplane.centroid_ / static_cast<float>(filtered_gplane.points_.size());
  // D - Bounding polygon
#if VERBOSE == 1
  t_->tick("lidarMapper::convexHull()");
#endif
  QuickConvexHull::process(filtered_gplane, out_groundplane);
#if VERBOSE == 1
  t_->tock();
#endif
  for (auto& pt : out_groundplane.points_)
  {
    pt = pt * tf;
  }
  for (auto& pt : out_groundplane.extremas_)
  {
    pt = pt * tf;
  }
  out_groundplane.centroid_ = out_groundplane.centroid_ * tf;
  // E - Check plane consistency (set to null if not consistent)
  if (out_groundplane.area_ < 4.)
  {
    out_groundplane = SemiPlane();
  }

  // -------------------------------------------------------------------------------
  // ----- Mark raw ground points
  // -------------------------------------------------------------------------------
  Plane non_flat_ground;
#if VERBOSE == 1
  t_->tick("lidarMapper::groundRemoval()");
#endif
  groundRemoval(transformed_pcl, non_flat_ground);
#if VERBOSE == 1
  t_->tock();
#endif
  for (const auto& index : non_flat_ground.indexes_)
  {
    int i = static_cast<int>(index.x_);
    int j = static_cast<int>(index.y_);

    ground_mat_(i, j) = 1;
    label_mat_(i, j) = -1;
  }

  // - Planes that are not the ground
  std::vector<PlanePoint> cloud_seg;
#if VERBOSE == 1
  t_->tick("lidarMapper::cloudSegmentation()");
#endif
  cloudSegmentation(transformed_pcl, cloud_seg);
#if VERBOSE == 1
  t_->tock();
#endif

  //- Corners feature extraction
#if VERBOSE == 1
  t_->tick("lidarMapper::extractFeatures()");
#endif
  extractFeatures(cloud_seg, out_corners, out_planars);
#if VERBOSE == 1
  t_->tock();
#endif

  // - Convert local maps to base link
  for (auto& corner : out_corners)
  {
    corner.pos_ = corner.pos_ * tf;
  }
  for (auto& planar : out_planars)
  {
    planar.pos_ = planar.pos_ * tf;
  }
}
}  // namespace vineslam