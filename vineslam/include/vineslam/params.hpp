#pragma once

#include <iostream>
#include <vector>

namespace vineslam
{
struct Parameters
{
  // -----------------------------------
  // ------ System settings
  // -----------------------------------
  std::string world_frame_id_{};
  std::string base_frame_id_{};
  std::string odom_frame_id_{};
  std::string lidar_sensor_frame_{};
  std::string imu_sensor_frame_{};

  // -----------------------------------
  // ------ System flags
  // -----------------------------------
  bool use_lidar_features_{};
  bool use_vertical_planes_{};
  bool use_ground_plane_{};
  bool use_gps_{};
  bool use_gps_altitude_{};
  bool use_gps_heading_{};
  bool use_imu_{};
  bool use_gyroscope_{};
  uint16_t publish_level_{};
  uint16_t initialization_type_{};

  // -----------------------------------
  // ------ Map origin - datum
  // -----------------------------------
  double map_datum_lat_{};
  double map_datum_long_{};
  double map_datum_alt_{};

  // -----------------------------------
  // ------ Heading offset (if we are using GNSS heading)
  // -----------------------------------
  double heading_offset_{};

  // -----------------------------------
  // ------ Roll/Pitch offset (if we are using IMU)
  // -----------------------------------
  double imu_roll_offset_{};
  double imu_pitch_offset_{};

  // -----------------------------------
  // ------ Robot dimensions
  // -----------------------------------
  float robot_dim_x_;
  float robot_dim_y_;
  float robot_dim_z_;

  // -----------------------------------
  // ------ Multi-layer mapping parameters
  // -----------------------------------
  bool autogen_topological_map_{};
  float gridmap_origin_x_{};
  float gridmap_origin_y_{};
  float gridmap_origin_z_{};
  float gridmap_width_{};
  float gridmap_lenght_{};
  float gridmap_height_{};
  float gridmap_resolution_{};
  float topological_map_dim_x_{};
  float topological_map_dim_y_{};
  float topological_map_dim_square_size_{};
  std::string map_output_folder_;
  std::string map_input_file_;
  std::string elevation_map_input_file_;
  std::string topological_map_input_file_;
  std::string grid_map_files_folder_;
  std::string topological_map_folder_;

  // -----------------------------------
  // ------ Particle filter parameters
  // -----------------------------------
  int number_particles_{};
  float sigma_xx_{};
  float sigma_yy_{};
  float sigma_zz_{};
  float sigma_RR_{};
  float sigma_PP_{};
  float sigma_YY_{};


  // -----------------------------------
  // ------ METHODS
  // -----------------------------------
  Parameters() = default;
  Parameters& operator = (const Parameters& p)
  {
    return *this;
  }
};

}  // namespace vineslam