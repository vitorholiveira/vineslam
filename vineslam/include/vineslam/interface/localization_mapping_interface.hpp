#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <vineslam/mapping/occupancy_map.hpp>
#include <vineslam/feature/three_dimensional.hpp>

#include <opencv2/core.hpp>

namespace vineslam
{
// Look up table for direct topological node indexing, and its corresponding information
// All this information is declared here since this class does the interface between the localization and mapping
// Thus, all the necessary information declared on mapping and required in localization is declared here
struct LookUpTable
{
  uint32_t* look_up_table_;
  int look_up_table_size_;
  int look_up_table_cols_;

  float look_up_table_res_;
  float look_up_table_min_x_;
  float look_up_table_min_y_;
};

// *************************************************************************
// ****** Convert the topological map into a double pointer array **********
// -------------------------------------------------------------------------
//  node (1),  node (2),              ...                      ,    node (N)
// | allocated , deallocated ,                                 , allocated |
// |   (0/1)   ,     --      ,                                 ,    (0/1)  |
// |   (0/1)   ,     --      ,                                 ,    (0/1)  |
// |   (0/1)   ,     --      ,                                 ,    (0/1)  |
// |   (0/1)   ,     --      ,                                 ,    (0/1)  |
// |   (0/1)   ,     --      ,                                 ,    (0/1)  |
// |   (0/1)   ,     --      ,                                 ,    (0/1)  |
// -------------------------------------------------------------------------
// *************************************************************************
class LocalizationMappingInterface
{
public:
  // Class constructor
  LocalizationMappingInterface(const int& number_of_nodes);

  // Functions to allocate new nodes, either with features, or empty nodes
  void allocateNode(const int& node_number, const std::vector<Corner>& corners, const float& cx, const float& cy,
                    const float& angle, OccupancyMap* map);
  void allocateNode(const int& node_number, const std::vector<Planar>& planars, const float& cx, const float& cy,
                    const float& angle, OccupancyMap* map);
  void allocateNode(const int& node_number, const std::vector<Corner>& corners, const std::vector<Planar>& planars,
                    const float& cx, const float& cy, const float& angle, OccupancyMap* map);
  void allocateNode(const int& node_number, const float& cx, const float& cy, const float& angle, OccupancyMap* map);

  // Function to deallocate a node
  void deallocateNode(const int& node_number, OccupancyMap* map);

  // Update cell of a given node at a given location
  bool updateCell(const int& node_number, const uint8_t& val, const Corner& corner, OccupancyMap* map);
  bool updateCell(const int& node_number, const uint8_t& val, const Planar& planar, OccupancyMap* map);

  // Return a cell state given the 3D point coordinates
  uint8_t getCellState(const int& node_number, const Corner& corner, OccupancyMap* map);
  uint8_t getCellState(const int& node_number, const Planar& planar, OccupancyMap* map);

  // Called from the topological map class to set the lookup table to direct access to node numbers from point
  // coordinates
  void setLookUpTable(const cv::Mat& look_up_table_img, const float& look_up_table_min_x,
                      const float& look_up_table_min_y, const float& look_up_table_res);

  // Print a report of the map's info to the console
  void printReport();

  // Map and sub-map sizes and information
  uint8_t** array_;
  uint32_t* sizes_;
  float* info_;

  // Number of nodes of the map
  int number_of_nodes_;

  // Topological node indexing lookup table
  LookUpTable look_up_table_data_;

private:
  // Flatten 3D to 1D
  int to1D(const float& x, const float& y, const float& z, const float& origin_x, const float& origin_y,
           const float& origin_z, const float& resolution, const float& resolution_z, const float& width,
           const float& length);

  // Set the ith bit on an uint8_t
  void setBit(const int& pos, const uint8_t& val, uint8_t& byte);

  // Get the ith bit from an uint8_t
  void getBit(const int& pos, uint8_t& val, const uint8_t& byte);
};
}  // namespace vineslam