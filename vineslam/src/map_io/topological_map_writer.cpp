#include "../include/vineslam/map_io/topological_map_writer.hpp"

namespace vineslam
{
TopologicalMapWriter::TopologicalMapWriter(const Parameters& params)
{
  file_path_ = params.topological_map_input_file_;
}

void TopologicalMapWriter::write(const Parameters& params, TopologicalMap* topological_map)
{
  // Create file
  std::ofstream xmlfile;
  xmlfile.open(file_path_);

  // Write header
  xmlfile << s_header_ << s_endl_ << s_endl_;
  // Write grid map file path
  xmlfile << open(s_map_file_) << "" << close(s_map_file_) << s_endl_ << s_endl_; // TODO (André Aguiar): is this even needed??

  // Iterate over all the vertexes on the graph
  std::vector<vertex_t> graph_vertexes = topological_map->graph_vertexes_;
  for (size_t i = 0; i < graph_vertexes.size(); i++)
  {
    // Write the vertex information to the file
    xmlfile << open(s_vertex_) << s_endl_;
    xmlfile << s_tab_ << open(s_index_) << topological_map->map_[graph_vertexes[i]].index_ << close(s_index_) << s_endl_;
    xmlfile << s_tab_ << open(s_circle_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_x_) << topological_map->map_[graph_vertexes[i]].center_.x_ << close(s_x_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_y_) << topological_map->map_[graph_vertexes[i]].center_.y_ << close(s_y_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_radius_) << (params.topological_map_dim_square_size_ / 2.0) << close(s_radius_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_lat_) << topological_map->map_[graph_vertexes[i]].center_.lat_ << close(s_lat_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_lon_) << topological_map->map_[graph_vertexes[i]].center_.lon_ << close(s_lon_) << s_endl_;
    xmlfile << s_tab_ << close(s_circle_) << s_endl_;
    xmlfile << s_tab_ << open(s_rectangle_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_x1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].x_ << close(s_x1_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_y1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].y_ << close(s_y1_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_x2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].x_ << close(s_x2_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_y2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].y_ << close(s_y2_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_lat_1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].lat_ << close(s_lat_1_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_lon_1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].lon_ << close(s_lon_1_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_lat_2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].lat_ << close(s_lat_2_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_lon_2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].lon_ << close(s_lon_2_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_x1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].x_ << close(s_rotated_x1_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_y1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].y_ << close(s_rotated_y1_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_x2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].x_ << close(s_rotated_x2_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_y2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].y_ << close(s_rotated_y2_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_x3_) << topological_map->map_[graph_vertexes[i]].rectangle_[2].x_ << close(s_rotated_x3_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_y3_) << topological_map->map_[graph_vertexes[i]].rectangle_[2].y_ << close(s_rotated_y3_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_x4_) << topological_map->map_[graph_vertexes[i]].rectangle_[3].x_ << close(s_rotated_x4_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_y4_) << topological_map->map_[graph_vertexes[i]].rectangle_[3].y_ << close(s_rotated_y4_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lat_1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].lat_ << close(s_rotated_lat_1_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lon_1_) << topological_map->map_[graph_vertexes[i]].rectangle_[0].lon_ << close(s_rotated_lon_1_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lat_2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].lat_ << close(s_rotated_lat_2_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lon_2_) << topological_map->map_[graph_vertexes[i]].rectangle_[1].lon_ << close(s_rotated_lon_2_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lat_3_) << topological_map->map_[graph_vertexes[i]].rectangle_[2].lat_ << close(s_rotated_lat_3_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lon_3_) << topological_map->map_[graph_vertexes[i]].rectangle_[2].lon_ << close(s_rotated_lon_3_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lat_4_) << topological_map->map_[graph_vertexes[i]].rectangle_[3].lat_ << close(s_rotated_lat_4_) << s_endl_;
    xmlfile << std::setprecision(10) << s_tab_ << s_tab_ << open(s_rotated_lon_4_) << topological_map->map_[graph_vertexes[i]].rectangle_[3].lon_ << close(s_rotated_lon_4_) << s_endl_;
    xmlfile << s_tab_ << s_tab_ << open(s_rotated_angle_) << topological_map->map_[graph_vertexes[i]].rectangle_orientation_ << close(s_rotated_angle_) << s_endl_;
    xmlfile << s_tab_ << close(s_rectangle_) << s_endl_;
    xmlfile << close(s_vertex_) << s_endl_;
  }

  // Write edges to the file
  xmlfile << open(s_edges_) << s_endl_;
  // TODO (André Aguiar): we will need to write the edges when we use them for localization/mapping
  xmlfile << close(s_edges_) << s_endl_;

  // Close and save the file
  xmlfile.close();
}

// ---------------------------------------------------------------
// ----- Writer and parser aux functions
// ---------------------------------------------------------------

std::string TopologicalMapWriter::open(const std::string& tag)
{
  std::string out = "<" + tag + ">";
  return out;
}
std::string TopologicalMapWriter::close(const std::string& tag)
{
  std::string out = "</" + tag + ">";
  return out;
}
}  // namespace vineslam