#pragma once

#include <iostream>
#include <ctime>
#include <fstream>

#include <vineslam/mapping/topological_map.hpp>
#include <vineslam/params.hpp>
#include <vineslam/map_io/map_writer.hpp>

namespace vineslam
{
class TopologicalMapWriter
{
public:
  TopologicalMapWriter(const Parameters& params);

  // Receives the occupancy grid map and writes it to a xml file
  void write(const Parameters& params, TopologicalMap* topological_map);

  // Tags to parse the file
  const std::string s_header_{ "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" };
  const std::string s_map_file_{ "map" };
  const std::string s_tab_{ "   " };
  const std::string s_endl_{ "\n" };
  const std::string s_vertex_{ "vertex" };
  const std::string s_index_{ "index" };
  const std::string s_circle_{ "circle" };
  const std::string s_lat_{ "lat" };
  const std::string s_lon_{ "lon" };
  const std::string s_x_{ "x" };
  const std::string s_y_{ "y" };
  const std::string s_radius_{ "radius" };
  const std::string s_rectangle_{ "rectangle" };
  const std::string s_rotated_rectangle_{ "rotated_rectangle" };
  const std::string s_x1_{ "x1" };
  const std::string s_y1_{ "y1" };
  const std::string s_x2_{ "x2" };
  const std::string s_y2_{ "y2" };
  const std::string s_lat_1_{ "lat_1" };
  const std::string s_lon_1_{ "lon_1" };
  const std::string s_lat_2_{ "lat_2" };
  const std::string s_lon_2_{ "lon_2" };
  const std::string s_rotated_x1_{ "rotated_x1" };
  const std::string s_rotated_y1_{ "rotated_y1" };
  const std::string s_rotated_x2_{ "rotated_x2" };
  const std::string s_rotated_y2_{ "rotated_y2" };
  const std::string s_rotated_x3_{ "rotated_x3" };
  const std::string s_rotated_y3_{ "rotated_y3" };
  const std::string s_rotated_x4_{ "rotated_x4" };
  const std::string s_rotated_y4_{ "rotated_y4" };
  const std::string s_rotated_lat_1_{ "rotated_lat_1" };
  const std::string s_rotated_lon_1_{ "rotated_lon_1" };
  const std::string s_rotated_lat_2_{ "rotated_lat_2" };
  const std::string s_rotated_lon_2_{ "rotated_lon_2" };
  const std::string s_rotated_lat_3_{ "rotated_lat_3" };
  const std::string s_rotated_lon_3_{ "rotated_lon_3" };
  const std::string s_rotated_lat_4_{ "rotated_lat_4" };
  const std::string s_rotated_lon_4_{ "rotated_lon_4" };
  const std::string s_rotated_angle_{ "rotated_angle" };
  const std::string s_edges_{ "edges" };
  const std::string s_edge_{ "edge" };
  const std::string s_v1_{ "v1" };
  const std::string s_v2_{ "v2" };

private:
  // Writer and parser auxiliary functions
  std::string open(const std::string& tag);
  std::string close(const std::string& tag);

  // Input parameters
  std::string file_path_;
};
}  // namespace vineslam
