#pragma once

#include <vineslam/mapping/lidar_mapping.hpp>

namespace vineslam
{
struct SegPCL
{
  std::vector<bool> is_ground;
  std::vector<int> start_col_idx;
  std::vector<int> end_col_idx;
  std::vector<int> col_idx;
  std::vector<float> range;
};

struct smoothness_t
{
  float value;
  size_t idx;
};

struct by_value
{
  bool operator()(smoothness_t const& left, smoothness_t const& right)
  {
    return left.value < right.value;
  }
};

class OmnidirectionalMapper : public LidarMapper
{
public:
  OmnidirectionalMapper(const Parameters& params, const std::string& lidar_type);

  // -------------------------------------------------------------------------------
  // ---- 3D pointcloud feature map
  // -------------------------------------------------------------------------------
  // Builds local map given the current 3D point cloud - for velodyne
  void localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners, std::vector<Planar>& out_planars,
                std::vector<SemiPlane>& out_planes, SemiPlane& out_groundplane);
  void localMap(const std::vector<Point>& pcl, std::vector<Corner>& out_corners, std::vector<Planar>& out_planars,
                SemiPlane& out_groundplane);

  // 3D cloud feature parameters
  float planes_th_{};
  float ground_th_{};
  float edge_threshold_{};
  float planar_threshold_{};
  int picked_num_{};
  int vertical_scans_{};
  int horizontal_scans_{};
  int ground_scan_idx_{};
  int segment_valid_point_num_{};
  int segment_valid_line_num_{};
  float vertical_angle_bottom_{};
  float ang_res_x_{};
  float ang_res_y_{};

private:
  // Method to reset all the global variables and members
  void reset();

  // Method that extracts the ground plane of an input point cloud
  void groundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl);
  void flatGroundRemoval(const std::vector<Point>& in_pts, Plane& out_pcl);

  // Cloud generic plane segmentation
  void cloudSegmentation(const std::vector<Point>& in_pts, std::vector<PlanePoint>& cloud_seg);

  // Label a segment of a 3D point cloud
  void labelComponents(const int& row, const int& col, int& label);

  // Extract a couple of semiplanes
  void extractHighLevelPlanes(const std::vector<Point>& in_plane_pts, const SemiPlane& ground_plane,
                              std::vector<SemiPlane>& out_planes);
  bool checkPlaneConsistency(const SemiPlane& plane, const SemiPlane& ground_plane);

  // 3D feature extraction from a point cloud
  void extractFeatures(const std::vector<PlanePoint>& in_plane_pts, std::vector<Corner>& out_corners,
                       std::vector<Planar>& out_planars);

  // Cloud segmentation matrices
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> range_mat_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> ground_mat_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> label_mat_;
  // Cloud segmentation & feature extraction structure
  SegPCL seg_pcl_;
};
}  // namespace vineslam