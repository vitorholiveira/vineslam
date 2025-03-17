#include <vineslam/interface/localization_mapping_interface.hpp>

namespace vineslam
{
LocalizationMappingInterface::LocalizationMappingInterface(const int& number_of_nodes)
  : number_of_nodes_(number_of_nodes)
{
  // Allocate array memory
  array_ = (uint8_t**)malloc(sizeof(uint8_t*) * number_of_nodes);
  sizes_ = (uint32_t*)malloc(sizeof(uint32_t) * number_of_nodes);
  info_ = (float*)malloc(sizeof(float) * number_of_nodes * 10);  // (cx, cy, theta, origin_x, origin_y, origin_z, res,
                                                                 // res_z, width, length)
  for (int i = 0; i < number_of_nodes; i++)
  {
    sizes_[i] = 0;             // all nodes have size 0 at the beginning
    info_[i * 10 + 0] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 1] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 2] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 3] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 4] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 5] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 6] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 7] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 8] = -1.0;  // deallocated nodes have this default value for information
    info_[i * 10 + 9] = -1.0;  // deallocated nodes have this default value for information
  }
}

void LocalizationMappingInterface::setLookUpTable(const cv::Mat& look_up_table_img, const float& look_up_table_min_x,
                                                  const float& look_up_table_min_y, const float& look_up_table_res)
{
  look_up_table_data_.look_up_table_min_x_ = look_up_table_min_x;
  look_up_table_data_.look_up_table_min_y_ = look_up_table_min_y;
  look_up_table_data_.look_up_table_res_ = look_up_table_res;
  look_up_table_data_.look_up_table_cols_ = look_up_table_img.cols;
  look_up_table_data_.look_up_table_size_ = look_up_table_img.cols * look_up_table_img.rows;
  look_up_table_data_.look_up_table_ = (uint32_t*)malloc(look_up_table_data_.look_up_table_size_ * sizeof(uint32_t));
  for (int x = 0; x < look_up_table_img.cols; x++)
  {
    for (int y = 0; y < look_up_table_img.rows; y++)
    {
      look_up_table_data_.look_up_table_[x + y * look_up_table_img.cols] = look_up_table_img.at<ushort>(y, x) - 1;
    }
  }
}

int LocalizationMappingInterface::to1D(const float& x, const float& y, const float& z, const float& origin_x,
                                       const float& origin_y, const float& origin_z, const float& resolution,
                                       const float& resolution_z, const float& width, const float& length)
{
  int ii = static_cast<int>(std::round(x / resolution + 0.49));
  int jj = static_cast<int>(std::round(y / resolution + 0.49));
  int xx = ii - static_cast<int>(std::round(origin_x / resolution + 0.49));
  int yy = jj - static_cast<int>(std::round(origin_y / resolution + 0.49));
  int zz = static_cast<int>(std::round((z - origin_z) / resolution_z));
  return (xx + (static_cast<int>(std::round(width / resolution + 0.49)) *
                (yy + (zz * static_cast<int>(std::round(length / resolution + 0.49))))));
}

void LocalizationMappingInterface::setBit(const int& pos, const uint8_t& val, uint8_t& byte)
{
  byte |= val << pos;
}

void LocalizationMappingInterface::getBit(const int& pos, uint8_t& val, const uint8_t& byte)
{
  val = (byte >> pos) & 1U;
}

void LocalizationMappingInterface::allocateNode(const int& node_number, const std::vector<Corner>& corners,
                                                const float& cx, const float& cy, const float& angle, OccupancyMap* map)
{
  // Allocate node memory
  allocateNode(node_number, cx, cy, angle, map);

  // Set the node data
  for (const auto& corner : corners)
  {
    int idx = to1D(corner.pos_.x_, corner.pos_.y_, corner.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(idx / 8);
    if (byte_number >= 0 && byte_number < (sizes_[node_number] / 2))
    {
      int bit_number = idx % 8;
      setBit(bit_number, static_cast<uint8_t>(1), array_[node_number][byte_number]);
    }
  }
}

void LocalizationMappingInterface::allocateNode(const int& node_number, const std::vector<Planar>& planars,
                                                const float& cx, const float& cy, const float& angle, OccupancyMap* map)
{
  // Allocate node memory
  allocateNode(node_number, cx, cy, angle, map);

  // Set the node data
  for (const auto& planar : planars)
  {
    int idx = to1D(planar.pos_.x_, planar.pos_.y_, planar.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(sizes_[node_number] / 2) + static_cast<int>(idx / 8);
    if (byte_number >= static_cast<int>(sizes_[node_number] / 2) && byte_number < sizes_[node_number])
    {
      int bit_number = idx % 8;
      setBit(bit_number, static_cast<uint8_t>(1), array_[node_number][byte_number]);
    }
  }
}

void LocalizationMappingInterface::allocateNode(const int& node_number, const std::vector<Corner>& corners,
                                                const std::vector<Planar>& planars, const float& cx, const float& cy,
                                                const float& angle, OccupancyMap* map)
{
  // Allocate node memory
  allocateNode(node_number, cx, cy, angle, map);

  // Set the node data
  for (const auto& planar : planars)
  {
    int idx = to1D(planar.pos_.x_, planar.pos_.y_, planar.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(sizes_[node_number] / 2) + static_cast<int>(idx / 8);
    if (byte_number >= static_cast<int>(sizes_[node_number] / 2) && byte_number < sizes_[node_number])
    {
      int bit_number = idx % 8;
      setBit(bit_number, static_cast<uint8_t>(1), array_[node_number][byte_number]);
    }
  }
  for (const auto& corner : corners)
  {
    int idx = to1D(corner.pos_.x_, corner.pos_.y_, corner.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(idx / 8);
    if (byte_number >= 0 && byte_number < (sizes_[node_number] / 2))
    {
      int bit_number = idx % 8;
      setBit(bit_number, static_cast<uint8_t>(1), array_[node_number][byte_number]);
    }
  }
}

void LocalizationMappingInterface::allocateNode(const int& node_number, const float& cx, const float& cy,
                                                const float& angle, OccupancyMap* map)
{
  int size = (map->width_ / map->resolution_) * (map->lenght_ / map->resolution_) * (map->height_ / map->resolution_z_);
  int size_with_bits = (static_cast<int>(size / 8) + 1) * 2;
  array_[node_number] = (uint8_t*)malloc(sizeof(uint8_t) * size_with_bits);
  sizes_[node_number] = size_with_bits;
  info_[node_number * 10 + 0] = cx;
  info_[node_number * 10 + 1] = cy;
  info_[node_number * 10 + 2] = angle;
  info_[node_number * 10 + 3] = map->origin_.x_;
  info_[node_number * 10 + 4] = map->origin_.y_;
  info_[node_number * 10 + 5] = map->origin_.z_;
  info_[node_number * 10 + 6] = map->resolution_;
  info_[node_number * 10 + 7] = map->resolution_z_;
  info_[node_number * 10 + 8] = map->width_;
  info_[node_number * 10 + 9] = map->lenght_;
  for (int i = 0; i < size_with_bits; i++)  // here we write directly 0 in all bytes (do not need to set them bitwise)
  {
    array_[node_number][i] = static_cast<uint8_t>(0);
  }
}

void LocalizationMappingInterface::deallocateNode(const int& node_number, OccupancyMap* map)
{
  free(array_[node_number]);
  sizes_[node_number] = 0;            // node as now size 0 again
  info_[node_number * 3 + 0] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 1] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 2] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 3] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 4] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 5] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 6] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 7] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 8] = -1.0;  // deallocated nodes have this default value for information
  info_[node_number * 3 + 9] = -1.0;  // deallocated nodes have this default value for information
}

bool LocalizationMappingInterface::updateCell(const int& node_number, const uint8_t& val, const Corner& corner,
                                              OccupancyMap* map)
{
  if (sizes_[node_number] == 0)
  {
#if VERBOSE == 1
    std::cout << "LocalizationMappingInterface::updateCell() - Trying to update cell of a non allocated node."
              << std::endl;
#endif
    return false;
  }
  else
  {
    int idx = to1D(corner.pos_.x_, corner.pos_.y_, corner.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(idx / 8);
    if (byte_number >= 0 && byte_number < (sizes_[node_number] / 2))
    {
      int bit_number = idx % 8;
      setBit(bit_number, val, array_[node_number][byte_number]);

      return true;
    }
    else
    {
#if VERBOSE == 1
      std::cout << "LocalizationMappingInterface::updateCell() - Access to the localization and mapping interface out "
                   "of bounds."
                << std::endl;
#endif
      return false;
    }
  }
}

bool LocalizationMappingInterface::updateCell(const int& node_number, const uint8_t& val, const Planar& planar,
                                              OccupancyMap* map)
{
  if (sizes_[node_number] == 0)
  {
#if VERBOSE == 1
    std::cout << "LocalizationMappingInterface::updateCell() - Trying to update cell of a non allocated node."
              << std::endl;
#endif
    return false;
  }
  else
  {
    int idx = to1D(planar.pos_.x_, planar.pos_.y_, planar.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(sizes_[node_number] / 2) + static_cast<int>(idx / 8);
    if (byte_number >= static_cast<int>(sizes_[node_number] / 2) && byte_number < sizes_[node_number])
    {
      int bit_number = idx % 8;
      setBit(bit_number, val, array_[node_number][byte_number]);

      return true;
    }
    else
    {
#if VERBOSE == 1
      std::cout << "LocalizationMappingInterface::updateCell() - Access to the localization and mapping interface out "
                   "of bounds."
                << std::endl;
#endif
      return false;
    }
  }
}

uint8_t LocalizationMappingInterface::getCellState(const int& node_number, const Corner& corner, OccupancyMap* map)
{
  uint8_t val = 0;
  if (sizes_[node_number] > 0)
  {
    int idx = to1D(corner.pos_.x_, corner.pos_.y_, corner.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(idx / 8);
    if (byte_number >= 0 && byte_number < (sizes_[node_number] / 2))
    {
      int bit_number = idx % 8;
      getBit(bit_number, val, array_[node_number][byte_number]);
    }
  }
  return val;
}

uint8_t LocalizationMappingInterface::getCellState(const int& node_number, const Planar& planar, OccupancyMap* map)
{
  uint8_t val = 0;
  if (sizes_[node_number] > 0)
  {
    int idx = to1D(planar.pos_.x_, planar.pos_.y_, planar.pos_.z_, map->origin_.x_, map->origin_.y_, map->origin_.z_,
                   map->resolution_, map->resolution_z_, map->width_, map->lenght_);

    int byte_number = static_cast<int>(sizes_[node_number] / 2) + static_cast<int>(idx / 8);
    if (byte_number >= static_cast<int>(sizes_[node_number] / 2) && byte_number < sizes_[node_number])
    {
      int bit_number = idx % 8;
      getBit(bit_number, val, array_[node_number][byte_number]);
    }
  }
  return val;
}

void LocalizationMappingInterface::printReport()
{
  std::vector<int> active_nodes;
  int number_nodes = 0;
  int mem_used = 0;

  for (int i = 0; i < number_of_nodes_; i++)
  {
    if (sizes_[i] > 0)
    {
      active_nodes.push_back(i);
      mem_used += (sizes_[i] * sizeof(uint8_t));
      number_nodes++;
    }
  }
  mem_used /= 1e6;

  std::cout << "LocalizationMappingInterface report:\n";
  std::cout << "   number of active nodes: " << number_nodes << "\n";
  std::cout << "   memory usage: " << mem_used << " MB\n";
}
}  // namespace vineslam
