#include <vineslam/mapping/lidar_mapping_livox.hpp>

namespace vineslam
{
LivoxMapper::LivoxMapper(const Parameters& params)
{
  // Set Livox configuration parameters
  pcl_data_save_index_ = 0;
  max_fov_ = 17;  // Edge of circle to main axis
  max_edge_polar_pos_ = 0;
  time_internal_pts_ = 1.0e-5;  // 10us = 1e-5
  cx_ = 0;
  cy_ = 0;
  if_save_pcd_file_ = 0;
  first_receive_time_ = -1;
  thr_corner_curvature_ = 0.1;
  thr_surface_curvature_ = 0.005;
  minimum_view_angle_ = 5;
  livox_min_allow_dis_ = 0.1;
  livox_min_sigma_ = 7e-4;

  // Set ground plane settings
  ground_th_ = static_cast<float>(3.) * DEGREE_TO_RAD;
  horizontal_scans_ = 1800;
  ground_scan_idx_ = 7;
  vertical_scans_ = 16;
  vertical_angle_bottom_ = static_cast<float>(15. + 0.1) * DEGREE_TO_RAD;
  ang_res_x_ = static_cast<float>(0.2) * DEGREE_TO_RAD;
  ang_res_y_ = static_cast<float>(2.) * DEGREE_TO_RAD;

  // Set robot dimensions for elevation map computation
  robot_dim_x_ = params.robot_dim_x_;
  robot_dim_y_ = params.robot_dim_y_;
  robot_dim_z_ = params.robot_dim_z_;

  // Set parameters object handler
  params_ = params;

  // Set previous robot pose handler
  prev_robot_pose_ = Pose(0, 0, 0, 0, 0, 0);
}

Pt_infos* LivoxMapper::findPtInfo(const Point& pt)
{
  map_pt_idx_it_ = map_pt_idx_.find(pt);
  if (map_pt_idx_it_ == map_pt_idx_.end())
  {
    assert(map_pt_idx_it_ != map_pt_idx_.end());  // else, there must be something error happened before.
  }
  return map_pt_idx_it_->second;
}

void LivoxMapper::getFeatures(std::vector<Point>& pc_corners, std::vector<Point>& pc_surface, std::vector<Point>& pc_full_res, float minimum_blur, float maximum_blur)
{
  int corner_num = 0;
  int surface_num = 0;
  int full_num = 0;
  pc_corners.resize(pts_info_vec_.size());
  pc_surface.resize(pts_info_vec_.size());
  pc_full_res.resize(pts_info_vec_.size());
  float maximum_idx = maximum_blur * pts_info_vec_.size();
  float minimum_idx = minimum_blur * pts_info_vec_.size();
  int pt_critical_rm_mask = e_pt_000 | e_pt_nan | e_pt_too_near;
  for (size_t i = 0; i < pts_info_vec_.size(); i++)
  {
    if (pts_info_vec_[i].idx_ > maximum_idx || pts_info_vec_[i].idx_ < minimum_idx)
      continue;

    if ((pts_info_vec_[i].pt_type_ & pt_critical_rm_mask) == 0)
    {
      if (pts_info_vec_[i].pt_label_ & e_label_corner)
      {
        //        if (pts_info_vec_[i].pt_type_ != e_pt_normal)
        //          continue;
        if (pts_info_vec_[i].depth_sq2_ < std::pow(30, 2))
        {
          pc_corners[corner_num] = raw_pts_vec_[i];
          // set_intensity( pc_corners[ corner_num ], e_I_motion_blur );
          pc_corners[corner_num].intensity_ = pts_info_vec_[i].time_stamp_;
          corner_num++;
        }
      }
      if (pts_info_vec_[i].pt_label_ & e_label_surface)
      {
        if (pts_info_vec_[i].depth_sq2_ < std::pow(1000, 2))
        {
          pc_surface[surface_num] = raw_pts_vec_[i];
          pc_surface[surface_num].intensity_ = float(pts_info_vec_[i].time_stamp_);
          // set_intensity( pc_surface[ surface_num ], e_I_motion_blur );
          surface_num++;
        }
      }
    }
    pc_full_res[full_num] = raw_pts_vec_[i];
    pc_full_res[full_num].intensity_ = pts_info_vec_[i].time_stamp_;
    full_num++;
  }

  // printf("Get_features , corner num = %d, suface num = %d, blur from %.2f~%.2f\r\n", corner_num, surface_num,
  // minimum_blur, maximum_blur);
  pc_corners.resize(corner_num);
  pc_surface.resize(surface_num);
  pc_full_res.resize(full_num);
}

void LivoxMapper::setIntensity(Point& pt, const E_intensity_type& i_type)
{
  Pt_infos* pt_info = findPtInfo(pt);
  switch (i_type)
  {
    case (e_I_raw):
      pt.intensity_ = pt_info->raw_intensity_;
      break;
    case (e_I_motion_blur):
      pt.intensity_ = ((float)pt_info->idx_) / (float)input_points_size_;
      assert(pt.intensity_ <= 1.0 && pt.intensity_ >= 0.0);
      break;
    case (e_I_motion_mix):
      pt.intensity_ = 0.1 * ((float)pt_info->idx_ + 1) / (float)input_points_size_ + (int)(pt_info->raw_intensity_);
      break;
    case (e_I_scan_angle):
      pt.intensity_ = pt_info->polar_angle_;
      break;
    case (e_I_curvature):
      pt.intensity_ = pt_info->curvature_;
      break;
    case (e_I_view_angle):
      pt.intensity_ = pt_info->view_angle_;
      break;
    case (e_I_time_stamp):
      pt.intensity_ = pt_info->time_stamp_;
      break;
    default:
      pt.intensity_ = ((float)pt_info->idx_ + 1) / (float)input_points_size_;
      break;
  }
  return;
}

void LivoxMapper::addMaskOfPoint(Pt_infos* pt_infos, const E_point_type& pt_type_, int neighbor_count)
{
  int idx = pt_infos->idx_;
  pt_infos->pt_type_ |= pt_type_;

  if (neighbor_count > 0)
  {
    for (int i = -neighbor_count; i < neighbor_count; i++)
    {
      idx = pt_infos->idx_ + i;

      if (i != 0 && (idx >= 0) && (idx < (int)pts_info_vec_.size()))
      {
        pts_info_vec_[idx].pt_type_ |= pt_type_;
      }
    }
  }
}

void LivoxMapper::evalPoint(Pt_infos* pt_info)
{
  if (pt_info->depth_sq2_ < livox_min_allow_dis_ * livox_min_allow_dis_)  // to close
  {
    addMaskOfPoint(pt_info, e_pt_too_near);
  }

  pt_info->sigma_ = pt_info->raw_intensity_ / pt_info->polar_dis_sq2_;

  if (pt_info->sigma_ < livox_min_sigma_)
  {
    addMaskOfPoint(pt_info, e_pt_reflectivity_low);
  }
}

template <typename T>
T LivoxMapper::vector_angle(const Eigen::Matrix<T, 3, 1>& vec_a, const Eigen::Matrix<T, 3, 1>& vec_b, int if_force_sharp_angle)
{
  T vec_a_norm = vec_a.norm();
  T vec_b_norm = vec_b.norm();
  if (vec_a_norm == 0 || vec_b_norm == 0)  // zero vector is pararrel to any vector.
  {
    return 0.0;
  }
  else
  {
    if (if_force_sharp_angle)
    {
      // return acos( abs( vec_a.dot( vec_b ) )*0.9999 / ( vec_a_norm * vec_b_norm ) );
      return acos(abs(vec_a.dot(vec_b)) / (vec_a_norm * vec_b_norm));
    }
    else
    {
      // return acos( (vec_a.dot(vec_b))*0.9999 / (vec_a_norm*vec_b_norm));
      return acos((vec_a.dot(vec_b)) / (vec_a_norm * vec_b_norm));
    }
  }
}

void LivoxMapper::computeFeatures()
{
  unsigned int pts_size = raw_pts_vec_.size();
  size_t curvature_ssd_size = 2;
  int critical_rm_point = e_pt_000 | e_pt_nan;
  float neighbor_accumulate_xyz[3] = { 0.0, 0.0, 0.0 };

  for (size_t idx = curvature_ssd_size; idx < pts_size - curvature_ssd_size; idx++)
  {
    if (pts_info_vec_[idx].pt_type_ & critical_rm_point)
    {
      continue;
    }

    // Compute curvature
    neighbor_accumulate_xyz[0] = 0.0;
    neighbor_accumulate_xyz[1] = 0.0;
    neighbor_accumulate_xyz[2] = 0.0;

    for (size_t i = 1; i <= curvature_ssd_size; i++)
    {
      if ((pts_info_vec_[idx + i].pt_type_ & e_pt_000) || (pts_info_vec_[idx - i].pt_type_ & e_pt_000))
      {
        if (i == 1)
        {
          pts_info_vec_[idx].pt_label_ |= e_label_near_zero;
        }
        else
        {
          pts_info_vec_[idx].pt_label_ = e_label_invalid;
        }
        break;
      }
      else if ((pts_info_vec_[idx + i].pt_type_ & e_pt_nan) || (pts_info_vec_[idx - i].pt_type_ & e_pt_nan))
      {
        if (i == 1)
        {
          pts_info_vec_[idx].pt_label_ |= e_label_near_nan;
        }
        else
        {
          pts_info_vec_[idx].pt_label_ = e_label_invalid;
        }
        break;
      }
      else
      {
        neighbor_accumulate_xyz[0] += raw_pts_vec_[idx + i].x_ + raw_pts_vec_[idx - i].x_;
        neighbor_accumulate_xyz[1] += raw_pts_vec_[idx + i].y_ + raw_pts_vec_[idx - i].y_;
        neighbor_accumulate_xyz[2] += raw_pts_vec_[idx + i].z_ + raw_pts_vec_[idx - i].z_;
      }
    }

    if (pts_info_vec_[idx].pt_label_ == e_label_invalid)
    {
      continue;
    }

    neighbor_accumulate_xyz[0] -= curvature_ssd_size * 2 * raw_pts_vec_[idx].x_;
    neighbor_accumulate_xyz[1] -= curvature_ssd_size * 2 * raw_pts_vec_[idx].y_;
    neighbor_accumulate_xyz[2] -= curvature_ssd_size * 2 * raw_pts_vec_[idx].z_;
    pts_info_vec_[idx].curvature_ = neighbor_accumulate_xyz[0] * neighbor_accumulate_xyz[0] + neighbor_accumulate_xyz[1] * neighbor_accumulate_xyz[1] + neighbor_accumulate_xyz[2] * neighbor_accumulate_xyz[2];

    // Compute plane angle
    Eigen::Matrix<float, 3, 1> vec_a(raw_pts_vec_[idx].x_, raw_pts_vec_[idx].y_, raw_pts_vec_[idx].z_);
    Eigen::Matrix<float, 3, 1> vec_b(raw_pts_vec_[idx + curvature_ssd_size].x_ - raw_pts_vec_[idx - curvature_ssd_size].x_, raw_pts_vec_[idx + curvature_ssd_size].y_ - raw_pts_vec_[idx - curvature_ssd_size].y_,
                                     raw_pts_vec_[idx + curvature_ssd_size].z_ - raw_pts_vec_[idx - curvature_ssd_size].z_);
    pts_info_vec_[idx].view_angle_ = vector_angle(vec_a, vec_b, 1) * 57.3;

    if (pts_info_vec_[idx].view_angle_ > minimum_view_angle_)
    {
      if (pts_info_vec_[idx].curvature_ < thr_surface_curvature_)
      {
        pts_info_vec_[idx].pt_label_ |= e_label_surface;
      }

      float sq2_diff = 0.1;

      if (pts_info_vec_[idx].curvature_ > thr_corner_curvature_)
      {
        if (pts_info_vec_[idx].depth_sq2_ <= pts_info_vec_[idx - curvature_ssd_size].depth_sq2_ && pts_info_vec_[idx].depth_sq2_ <= pts_info_vec_[idx + curvature_ssd_size].depth_sq2_)
        {
          if (abs(pts_info_vec_[idx].depth_sq2_ - pts_info_vec_[idx - curvature_ssd_size].depth_sq2_) < sq2_diff * pts_info_vec_[idx].depth_sq2_ ||
              abs(pts_info_vec_[idx].depth_sq2_ - pts_info_vec_[idx + curvature_ssd_size].depth_sq2_) < sq2_diff * pts_info_vec_[idx].depth_sq2_)
            pts_info_vec_[idx].pt_label_ |= e_label_corner;
        }
      }
    }
  }
}

int LivoxMapper::projectionScan3d2d(const std::vector<Point>& laser_cloud_in, std::vector<float>& scan_id_index)
{
  unsigned int pts_size = laser_cloud_in.size();
  pts_info_vec_.clear();
  pts_info_vec_.resize(pts_size);
  raw_pts_vec_.resize(pts_size);
  std::vector<int> edge_idx;
  std::vector<int> split_idx;
  scan_id_index.resize(pts_size);
  map_pt_idx_.clear();
  map_pt_idx_.reserve(pts_size);
  std::vector<int> zero_idx;

  input_points_size_ = 0;

  for (unsigned int idx = 0; idx < pts_size; idx++)
  {
    raw_pts_vec_[idx] = laser_cloud_in[idx];
    Pt_infos* pt_info = &pts_info_vec_[idx];
    map_pt_idx_.insert(std::make_pair(laser_cloud_in[idx], pt_info));
    pt_info->raw_intensity_ = laser_cloud_in[idx].intensity_;
    pt_info->idx_ = idx;
    pt_info->time_stamp_ = current_time_ + ((float)idx) * time_internal_pts_;
    last_maximum_time_stamp_ = pt_info->time_stamp_;
    input_points_size_++;

    if (!std::isfinite(laser_cloud_in[idx].x_) || !std::isfinite(laser_cloud_in[idx].y_) || !std::isfinite(laser_cloud_in[idx].z_))
    {
      addMaskOfPoint(pt_info, e_pt_nan);
      continue;
    }

    if (laser_cloud_in[idx].x_ == 0)
    {
      if (idx == 0)
      {
        // TODO: handle this case.

        pt_info->pt_2d_img_ << 0.01, 0.01;
        pt_info->polar_dis_sq2_ = 0.0001;
        addMaskOfPoint(pt_info, e_pt_000);
        // return 0;
      }
      else
      {
        pt_info->pt_2d_img_ = pts_info_vec_[idx - 1].pt_2d_img_;
        pt_info->polar_dis_sq2_ = pts_info_vec_[idx - 1].polar_dis_sq2_;
        addMaskOfPoint(pt_info, e_pt_000);
        continue;
      }
    }

    map_pt_idx_.insert(std::make_pair(laser_cloud_in[idx], pt_info));

    pt_info->depth_sq2_ = laser_cloud_in[idx].norm3D();

    pt_info->pt_2d_img_ << laser_cloud_in[idx].y_ / laser_cloud_in[idx].x_, laser_cloud_in[idx].z_ / laser_cloud_in[idx].x_;
    pt_info->polar_dis_sq2_ = pt_info->pt_2d_img_(0) * pt_info->pt_2d_img_(0) + pt_info->pt_2d_img_(1) * pt_info->pt_2d_img_(1);

    evalPoint(pt_info);

    if (pt_info->polar_dis_sq2_ > max_edge_polar_pos_)
    {
      addMaskOfPoint(pt_info, e_pt_circle_edge, 2);
    }

    // Split scans
    if (idx >= 1)
    {
      float dis_incre = pt_info->polar_dis_sq2_ - pts_info_vec_[idx - 1].polar_dis_sq2_;

      if (dis_incre > 0)  // far away from zero
      {
        pt_info->polar_direction_ = 1;
      }

      if (dis_incre < 0)  // move toward zero
      {
        pt_info->polar_direction_ = -1;
      }

      if (pt_info->polar_direction_ == -1 && pts_info_vec_[idx - 1].polar_direction_ == 1)
      {
        if (edge_idx.size() == 0 || (idx - split_idx[split_idx.size() - 1]) > 50)
        {
          split_idx.push_back(idx);
          edge_idx.push_back(idx);
          continue;
        }
      }

      if (pt_info->polar_direction_ == 1 && pts_info_vec_[idx - 1].polar_direction_ == -1)
      {
        if (zero_idx.size() == 0 || (idx - split_idx[split_idx.size() - 1]) > 50)
        {
          split_idx.push_back(idx);

          zero_idx.push_back(idx);
          continue;
        }
      }
    }
  }
  split_idx.push_back(pts_size - 1);

  int val_index = 0;
  int pt_angle_index = 0;
  float scan_angle = 0;
  int internal_size = 0;

  if (split_idx.size() < 6)  // minimum 3 petal of scan.
    return 0;

  for (int idx = 0; idx < (int)pts_size; idx++)
  {
    if (val_index < (int)split_idx.size() - 2)
    {
      if (idx == 0 || idx > split_idx[val_index + 1])
      {
        if (idx > split_idx[val_index + 1])
        {
          val_index++;
        }

        internal_size = split_idx[val_index + 1] - split_idx[val_index];

        if (pts_info_vec_[split_idx[val_index + 1]].polar_dis_sq2_ > 10000)
        {
          pt_angle_index = split_idx[val_index + 1] - (int)(internal_size * 0.20);
          scan_angle = atan2(pts_info_vec_[pt_angle_index].pt_2d_img_(1), pts_info_vec_[pt_angle_index].pt_2d_img_(0)) * 57.3;
          scan_angle = scan_angle + 180.0;
        }
        else
        {
          pt_angle_index = split_idx[val_index + 1] - (int)(internal_size * 0.80);
          scan_angle = atan2(pts_info_vec_[pt_angle_index].pt_2d_img_(1), pts_info_vec_[pt_angle_index].pt_2d_img_(0)) * 57.3;
          scan_angle = scan_angle + 180.0;
        }
      }
    }
    pts_info_vec_[idx].polar_angle_ = scan_angle;
    scan_id_index[idx] = scan_angle;
  }

  return split_idx.size() - 1;
}

// Split whole point cloud into scans.
void LivoxMapper::splitLaserScan(const int clutter_size, const std::vector<Point>& laser_cloud_in, const std::vector<float>& scan_id_index, std::vector<std::vector<Point>>& laser_cloud_scans)
{
  std::vector<std::vector<int>> pts_mask;
  laser_cloud_scans.resize(clutter_size);
  pts_mask.resize(clutter_size);
  Point point;
  int scan_idx = 0;

  for (unsigned int i = 0; i < laser_cloud_in.size(); i++)
  {
    point = laser_cloud_in[i];

    if (i > 0 && ((scan_id_index[i]) != (scan_id_index[i - 1])))
    {
      scan_idx = scan_idx + 1;
      if (scan_idx > pts_mask.size() - 1)
      {
        break;
      }
      pts_mask[scan_idx].reserve(5000);
    }

    laser_cloud_scans[scan_idx].push_back(point);
    pts_mask[scan_idx].push_back(pts_info_vec_[i].pt_type_);
  }
  laser_cloud_scans.resize(scan_idx);

  int remove_point_pt_type_ = e_pt_000 | e_pt_too_near | e_pt_nan;
  int scan_avail_num = 0;
  std::vector<std::vector<Point>> res_laser_cloud_scan;
  for (unsigned int i = 0; i < laser_cloud_scans.size(); i++)
  {
    scan_avail_num = 0;
    std::vector<Point> laser_clour_per_scan;
    for (unsigned int idx = 0; idx < laser_cloud_scans[i].size(); idx++)
    {
      if ((pts_mask[i][idx] & remove_point_pt_type_) == 0)
      {
        if (laser_cloud_scans[i][idx].x_ == 0)
        {
          assert(laser_cloud_scans[i][idx].x_ != 0);
          continue;
        }
        auto temp_pt = laser_cloud_scans[i][idx];
        if (map_pt_idx_.find(temp_pt) != map_pt_idx_.end())
        {
          setIntensity(temp_pt, default_return_intensity_type_);
          laser_clour_per_scan.push_back(temp_pt);
          scan_avail_num++;
        }
      }
    }

    if (scan_avail_num)
    {
      res_laser_cloud_scan.push_back(laser_clour_per_scan);
    }
  }
  laser_cloud_scans = res_laser_cloud_scan;
}

std::vector<std::vector<Point>> LivoxMapper::extractLaserFeatures(const std::vector<Point>& laser_cloud_in, double time_stamp)
{
  assert(time_stamp >= 0.0);
  if (time_stamp <= 0.0000001 || (time_stamp < last_maximum_time_stamp_))  // old firmware, without timestamp
  {
    current_time_ = time_stamp;  // last_maximum_time_stamp_;
  }
  else
  {
    current_time_ = time_stamp - first_receive_time_;
  }
  if (first_receive_time_ <= 0)
  {
    first_receive_time_ = time_stamp;
  }

  std::vector<std::vector<Point>> laser_cloud_scans, temp_laser_scans;
  std::vector<float> scan_id_index;
  laser_cloud_scans.clear();
  map_pt_idx_.clear();
  pts_info_vec_.clear();
  raw_pts_vec_.clear();

  int clutter_size = projectionScan3d2d(laser_cloud_in, scan_id_index);
  computeFeatures();
  if (clutter_size == 0)
  {
    return laser_cloud_scans;
  }
  else
  {
    splitLaserScan(clutter_size, laser_cloud_in, scan_id_index, laser_cloud_scans);
    return laser_cloud_scans;
  }
}

void LivoxMapper::flatGroundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl)
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

      //      if (range_mat_(i, j) == -1 || range_mat_(i + 1, j) == -1)
      //      {
      //        // no info to check, invalid points
      //        //        ground_mat_(i, j) = -1;
      //        continue;
      //      }

      float dX = upper_pt.x_ - lower_pt.x_;
      float dY = upper_pt.y_ - lower_pt.y_;
      float dZ = upper_pt.z_ - lower_pt.z_;

      float vertical_angle = std::atan2(dZ, std::sqrt(dX * dX + dY * dY + dZ * dZ));

      if (vertical_angle <= ground_th_ && std::fabs(lower_pt.z_) > laser2base_z_ / 2 && std::fabs(upper_pt.z_) > laser2base_z_ / 2)
      {
        out_pcl.points_.push_back(lower_pt);
        out_pcl.points_.push_back(upper_pt);
        out_pcl.indexes_.emplace_back(i, j);
        out_pcl.indexes_.emplace_back(i + 1, j);
      }
    }
  }
}

bool LivoxMapper::checkPlaneConsistency(const SemiPlane& plane, const SemiPlane& ground_plane)
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
  //    d1 = min_dist;
  //    if (std::fabs(d1 - d0) > 0.2 && d0 != 0)  // We found a hole in this case ...
  //    {
  //      return false;
  //    }
  //    else
  //    {
  //      pts.erase(pts.begin() + idx);
  //      d0 = d1;
  //    }
  //  }

  // C - Make sure that the plane is not horizontal
  float dot = Vec(plane.a_, plane.b_, plane.c_).dot(Vec(ground_plane.a_, ground_plane.b_, ground_plane.c_));
  if (std::fabs(dot) > 1.0)
  {
    return false;
  }

  return true;
}

void LivoxMapper::extractHighLevelPlanes(const std::vector<Point>& in_pts, const SemiPlane& ground_plane, std::vector<SemiPlane>& out_planes)
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
  std::vector<Plane> planes = {};
  Plane side_plane_a_filtered, side_plane_b_filtered;
  if (Ransac::process(side_plane_a.points_, side_plane_a_filtered, 128, 0.10, true) && side_plane_a_filtered.points_.size() < 7000 && side_plane_a_filtered.points_.size() > 75)  // prevent dense planes and slow convex hulls
  {
    side_plane_a_filtered.id_ = 0;
    planes.push_back(side_plane_a_filtered);
  }
  if (Ransac::process(side_plane_b.points_, side_plane_b_filtered, 128, 0.10, true) && side_plane_b_filtered.points_.size() < 7000 && side_plane_b_filtered.points_.size() > 75)  // prevent dense planes and slow convex hulls
  {
    side_plane_b_filtered.id_ = 1;
    planes.push_back(side_plane_b_filtered);
  }

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
    bool ch = QuickConvexHull::process(plane, l_semi_plane);
    if (ch && checkPlaneConsistency(l_semi_plane, ground_plane))
    {
      out_planes.push_back(l_semi_plane);
    }
  }
}

void LivoxMapper::localMap(const std::vector<Point>& pcl, const double& time_stamp, std::vector<Corner>& out_corners, std::vector<Planar>& out_planars, std::vector<SemiPlane>& out_planes, SemiPlane& out_groundplane)
{
  if (pcl.empty())
  {
    return;
  }

  // Build velodyne to base_link transformation matrix
  Pose tf_pose(laser2base_x_, laser2base_y_, laser2base_z_, laser2base_roll_, laser2base_pitch_, laser2base_yaw_);
  Tf tf;
  std::array<float, 9> tf_rot{};
  tf_pose.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ tf_pose.x_, tf_pose.y_, tf_pose.z_ }).inverse();

  // Extract livox features
  std::vector<std::vector<Point>> laser_cloud_scans = extractLaserFeatures(pcl, time_stamp);
  std::vector<Point> tmp_corners, tmp_planars, tmp_full;
  getFeatures(tmp_corners, tmp_planars, tmp_full);

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

    if (range > 50.0 || (std::fabs(l_pt.x_) < 0.9 && std::fabs(l_pt.y_) < 0.4))
    {
      continue;
    }

    size_t idx = column_idx + row_idx * horizontal_scans_;
    transformed_pcl[idx] = l_pt;
  }

  // Extract ground plane
  Plane unfiltered_gplane, filtered_gplane;
  flatGroundRemoval(transformed_pcl, unfiltered_gplane);
  // B - Filtering
  Ransac::process(unfiltered_gplane.points_, filtered_gplane, 100, 0.01, true);
  // C - Centroid calculation
  for (const auto& pt : filtered_gplane.points_)
  {
    filtered_gplane.centroid_ = filtered_gplane.centroid_ + pt;
  }
  filtered_gplane.centroid_ = filtered_gplane.centroid_ / static_cast<float>(filtered_gplane.points_.size());
  // D - Bounding polygon
  QuickConvexHull::process(filtered_gplane, out_groundplane);

  if (params_.use_vertical_planes_)
  {
    // Extract high level planes, and then convert them to semi-planes
    extractHighLevelPlanes(transformed_pcl, out_groundplane, out_planes);
  }

  // Convert features to VineSLAM type and project them to the base_link
  for (const auto& pt : tmp_corners)
  {
    Corner c(Point(pt.x_, pt.y_, pt.z_), 0);
    c.pos_ = c.pos_ * tf;
    out_corners.push_back(c);
  }
  for (const auto& pt : tmp_planars)
  {
    Planar p(Point(pt.x_, pt.y_, pt.z_), 0);
    p.pos_ = p.pos_ * tf;
    out_planars.push_back(p);
  }
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

void LivoxMapper::localMap(const std::vector<Point>& pcl, const double& time_stamp, std::vector<Corner>& out_corners, std::vector<Planar>& out_planars, SemiPlane& out_groundplane)
{
  if (pcl.empty())
  {
    return;
  }

  // Build velodyne to base_link transformation matrix
  Pose tf_pose(laser2base_x_, laser2base_y_, laser2base_z_, laser2base_roll_, laser2base_pitch_, laser2base_yaw_);
  Tf tf;
  std::array<float, 9> tf_rot{};
  tf_pose.toRotMatrix(tf_rot);
  tf = Tf(tf_rot, std::array<float, 3>{ tf_pose.x_, tf_pose.y_, tf_pose.z_ }).inverse();

  // Extract livox features
  std::vector<std::vector<Point>> laser_cloud_scans = extractLaserFeatures(pcl, time_stamp);
  std::vector<Point> tmp_corners, tmp_planars, tmp_full;
  getFeatures(tmp_corners, tmp_planars, tmp_full);

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

    if (range > 50.0 || (std::fabs(l_pt.x_) < 0.9 && std::fabs(l_pt.y_) < 0.4))
    {
      continue;
    }

    size_t idx = column_idx + row_idx * horizontal_scans_;
    transformed_pcl[idx] = l_pt;
  }

  // Extract ground plane
  Plane unfiltered_gplane, filtered_gplane;
  flatGroundRemoval(transformed_pcl, unfiltered_gplane);
  // B - Filtering
  Ransac::process(unfiltered_gplane.points_, filtered_gplane, 100, 0.01, true);
  // C - Centroid calculation
  for (const auto& pt : filtered_gplane.points_)
  {
    filtered_gplane.centroid_ = filtered_gplane.centroid_ + pt;
  }
  filtered_gplane.centroid_ = filtered_gplane.centroid_ / static_cast<float>(filtered_gplane.points_.size());
  // D - Bounding polygon
  QuickConvexHull::process(filtered_gplane, out_groundplane);

  // Convert features to VineSLAM type and project them to the base_link
  for (const auto& pt : tmp_corners)
  {
    Corner c(Point(pt.x_, pt.y_, pt.z_), 0);
    c.pos_ = c.pos_ * tf;
    out_corners.push_back(c);
  }
  for (const auto& pt : tmp_planars)
  {
    Planar p(Point(pt.x_, pt.y_, pt.z_), 0);
    p.pos_ = p.pos_ * tf;
    out_planars.push_back(p);
  }
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
}
}  // namespace vineslam