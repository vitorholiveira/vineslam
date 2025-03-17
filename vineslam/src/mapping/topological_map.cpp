#include <vineslam/mapping/topological_map.hpp>

namespace vineslam
{
TopologicalMap::TopologicalMap(const Parameters& params) : params_(params)
{
  is_initialized_ = false;
  init_function_called_ = false;

  // Initialize temporary occupancy grid map to store the features detected
  // while the topological map is not yet initialized
  Parameters l_params;
  l_params.gridmap_width_ = 70.0;
  l_params.gridmap_lenght_ = 70.0;
  l_params.gridmap_height_ = 15.0;
  l_params.gridmap_origin_x_ = -35.0;
  l_params.gridmap_origin_y_ = -35.0;
  l_params.gridmap_origin_z_ = -7.5;
  l_params.gridmap_resolution_ = 0.25;

  tmp_grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
}

void TopologicalMap::init(const double& heading, LocalizationMappingInterface* localization_mapping_interface, bool load_lookup_table_from_file)
{
  // Variables to initialize look-up table image (find dimensions and set resolution)
  look_up_table_max_x_ = 0;
  look_up_table_max_y_ = 0;
  look_up_table_min_x_ = std::numeric_limits<float>::max();
  look_up_table_min_y_ = std::numeric_limits<float>::max();

  // Initialize interface map
  localization_mapping_interface_ = localization_mapping_interface;

  // Mapping between rotated and non-rotated rectangles to allocate grid map memory
  for (size_t i = 0; i < graph_vertexes_.size(); i++)
  {
    map_[graph_vertexes_[i]].rectangle_orientation_ += (heading + M_PI_2);
    Point center(map_[graph_vertexes_[i]].center_.x_, map_[graph_vertexes_[i]].center_.y_, 0);

    Point p1(map_[graph_vertexes_[i]].rectangle_[0].x_, map_[graph_vertexes_[i]].rectangle_[0].y_, 0);
    Point p3(map_[graph_vertexes_[i]].rectangle_[2].x_, map_[graph_vertexes_[i]].rectangle_[2].y_, 0);
    Point out_p1, out_p3;

    alignPoint(p1, center, map_[graph_vertexes_[i]].rectangle_orientation_, out_p1);
    alignPoint(p3, center, map_[graph_vertexes_[i]].rectangle_orientation_, out_p3);

    map_[graph_vertexes_[i]].aligned_rectangle_.resize(4);
    map_[graph_vertexes_[i]].aligned_rectangle_[0].x_ = out_p1.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[0].y_ = out_p1.y_;
    map_[graph_vertexes_[i]].aligned_rectangle_[1].x_ = out_p1.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[1].y_ = out_p3.y_;
    map_[graph_vertexes_[i]].aligned_rectangle_[2].x_ = out_p3.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[2].y_ = out_p3.y_;
    map_[graph_vertexes_[i]].aligned_rectangle_[3].x_ = out_p3.x_;
    map_[graph_vertexes_[i]].aligned_rectangle_[3].y_ = out_p1.y_;

    // Set the class flags
    map_[graph_vertexes_[i]].has_file_ = false;

    // Compute look-up table boundaries
    if (map_[graph_vertexes_[i]].rectangle_[0].x_ < look_up_table_min_x_)
    {
      look_up_table_min_x_ = map_[graph_vertexes_[i]].rectangle_[0].x_;
    }
    if (map_[graph_vertexes_[i]].rectangle_[2].x_ < look_up_table_min_x_)
    {
      look_up_table_min_x_ = map_[graph_vertexes_[i]].rectangle_[2].x_;
    }
    if (map_[graph_vertexes_[i]].rectangle_[0].x_ > look_up_table_max_x_)
    {
      look_up_table_max_x_ = map_[graph_vertexes_[i]].rectangle_[0].x_;
    }
    if (map_[graph_vertexes_[i]].rectangle_[2].x_ > look_up_table_max_x_)
    {
      look_up_table_max_x_ = map_[graph_vertexes_[i]].rectangle_[2].x_;
    }
    if (map_[graph_vertexes_[i]].rectangle_[0].y_ < look_up_table_min_y_)
    {
      look_up_table_min_y_ = map_[graph_vertexes_[i]].rectangle_[0].y_;
    }
    if (map_[graph_vertexes_[i]].rectangle_[2].y_ < look_up_table_min_y_)
    {
      look_up_table_min_y_ = map_[graph_vertexes_[i]].rectangle_[2].y_;
    }
    if (map_[graph_vertexes_[i]].rectangle_[0].y_ > look_up_table_max_y_)
    {
      look_up_table_max_y_ = map_[graph_vertexes_[i]].rectangle_[0].y_;
    }
    if (map_[graph_vertexes_[i]].rectangle_[2].y_ > look_up_table_max_y_)
    {
      look_up_table_max_y_ = map_[graph_vertexes_[i]].rectangle_[2].y_;
    }
  }

  look_up_table_res_ = 0.25;
  if (!load_lookup_table_from_file)
  {
    // Create look-up table
    int size_x = std::abs(look_up_table_max_x_ - look_up_table_min_x_) / look_up_table_res_, size_y = std::abs(look_up_table_max_y_ - look_up_table_min_y_) / look_up_table_res_;
    look_up_table_img_ = cv::Mat::ones(cv::Size(size_x, size_y), CV_16UC1);

    // Fill look-up table
    float i = static_cast<float>(look_up_table_min_x_), j = static_cast<float>(look_up_table_min_y_);
    while (i < look_up_table_max_x_)
    {
      while (j < look_up_table_max_y_)
      {
        // Init vars
        int node = -1;
        float min_dist = std::numeric_limits<float>::max();
        bool found_solution = false;
        Point point(i, j, 0);

        // Go through every active vertex
        for (size_t k = 0; k < graph_vertexes_.size(); k++)
        {
          // Compute distance from the point to the center of the vertex
          Point center = Point(map_[graph_vertexes_[k]].center_.x_, map_[graph_vertexes_[k]].center_.y_, 0.);
          float dist = center.distanceXY(point);

          // Find the closest node
          if (dist < min_dist)
          {
            if (map_[graph_vertexes_[k]].aligned_rectangle_.size() != 4)
            {
              continue;
            }

            // Check if the point lies inside the rotated rectangle
            Point aligned_point;
            alignPoint(point, center, map_[graph_vertexes_[k]].rectangle_orientation_, aligned_point);
            // float dist_x = std::fabs(aligned_point.x_ - center.x_);
            // float dist_y = std::fabs(aligned_point.y_ - center.y_);
            float dist_x = std::fabs(aligned_point.x_ - ((map_[graph_vertexes_[k]].aligned_rectangle_[0].x_ + map_[graph_vertexes_[k]].aligned_rectangle_[2].x_) / 2.));
            float dist_y = std::fabs(aligned_point.y_ - ((map_[graph_vertexes_[k]].aligned_rectangle_[0].y_ + map_[graph_vertexes_[k]].aligned_rectangle_[2].y_) / 2.));
            float size_x = std::fabs(map_[graph_vertexes_[k]].aligned_rectangle_[0].x_ - map_[graph_vertexes_[k]].aligned_rectangle_[2].x_);
            float size_y = std::fabs(map_[graph_vertexes_[k]].aligned_rectangle_[0].y_ - map_[graph_vertexes_[k]].aligned_rectangle_[2].y_);

            // We found a possible node for the point
            if (dist_x <= (size_x / 2.) && dist_y <= (size_y / 2.))
            {
              min_dist = dist;
              node = k;
              found_solution = true;
            }
          }
        }

        if (found_solution)
        {
          int i_img = static_cast<int>((i - look_up_table_min_x_) / look_up_table_res_);
          int j_img = static_cast<int>((j - look_up_table_min_y_) / look_up_table_res_);
          if (i_img >= 0 && i_img < look_up_table_img_.cols && j_img >= 0 && j_img < look_up_table_img_.rows)
          {
            look_up_table_img_.at<ushort>(j_img, i_img) = node + 1;  // we add (1) just to use 0 as fault detector
          }
        }

        j += look_up_table_res_;
      }

      i += look_up_table_res_;
      j = static_cast<float>(look_up_table_min_y_);
    }

    // Save look up table image to a file
    cv::imwrite(params_.map_output_folder_ + "look_up_table.png", look_up_table_img_);
  }
  else
  {
    look_up_table_img_ = cv::imread(params_.map_output_folder_ + "look_up_table.png", -1);
    if (look_up_table_img_.empty())
    {
      std::cout << "\033[1;31mError reading the lookup table image\033[0m\n" << std::flush;
      exit(-1);
    }
  }

  // Copy lookup table to localization and mapping interface
  localization_mapping_interface_->setLookUpTable(look_up_table_img_, look_up_table_min_x_, look_up_table_min_y_, look_up_table_res_);

  // Store the temporary occupancy map into the graph-like structure
  std::vector<Planar> l_planars = tmp_grid_map_->getPlanars();
  std::vector<Corner> l_corners = tmp_grid_map_->getCorners();
  std::map<int, SemanticFeature> l_landmarks = tmp_grid_map_->getLandmarks();

  for (const auto& ft : l_planars)
  {
    directInsert(ft);  // direct insert since the features are persistence
                       // (they already passed through that test while being inserted
                       // on the temporary map)
  }
  for (const auto& ft : l_corners)
  {
    directInsert(ft);  // direct insert since the features are persistence
                       // (they already passed through that test while being inserted
                       // on the temporary map)
  }
  for (const auto& ft : l_landmarks)
  {
    insert(ft.second, ft.second.id_);
  }
  for (const auto& plane : tmp_grid_map_->planes_)
  {
    planes_.push_back(plane);
  }

  // Protect the access to these two variables
  mtx_.lock();

  // Set the class flags
  is_initialized_ = true;

  // Release the temporary occupancy grid map memory
  tmp_grid_map_->deallocateAllMem();
  delete tmp_grid_map_;
  tmp_grid_map_ = nullptr;

  mtx_.unlock();
  // ----------------------------------------
}

void TopologicalMap::polar2Enu(Geodetic* geodetic_converter)
{
  // Go through every vertex and compute its enu position on the map
  for (size_t i = 0; i < graph_vertexes_.size(); i++)
  {
    double n, e, d;
    Point corrected_point;

    // Resize rectangle array
    map_[graph_vertexes_[i]].rectangle_.resize(4);

    // Compute the enu location of the rectangle first vertex
    geodetic_converter->geodetic2ned(map_[graph_vertexes_[i]].rectangle_[0].lat_, map_[graph_vertexes_[i]].rectangle_[0].lon_, 0, n, e, d);
    map_[graph_vertexes_[i]].rectangle_[0].x_ = e;
    map_[graph_vertexes_[i]].rectangle_[0].y_ = n;

    // Compute the enu location of the rectangle first vertex
    geodetic_converter->geodetic2ned(map_[graph_vertexes_[i]].rectangle_[1].lat_, map_[graph_vertexes_[i]].rectangle_[1].lon_, 0, n, e, d);

    // Rotate the obtained point considering the gnss heading
    Point enu_c2(e, n, 0);
    map_[graph_vertexes_[i]].rectangle_[1].x_ = e;
    map_[graph_vertexes_[i]].rectangle_[1].y_ = n;

    // Compute the enu location of the rectangle first vertex
    geodetic_converter->geodetic2ned(map_[graph_vertexes_[i]].rectangle_[2].lat_, map_[graph_vertexes_[i]].rectangle_[2].lon_, 0, n, e, d);
    map_[graph_vertexes_[i]].rectangle_[2].x_ = e;
    map_[graph_vertexes_[i]].rectangle_[2].y_ = n;

    // Compute the enu location of the rectangle first vertex
    geodetic_converter->geodetic2ned(map_[graph_vertexes_[i]].rectangle_[3].lat_, map_[graph_vertexes_[i]].rectangle_[3].lon_, 0, n, e, d);
    map_[graph_vertexes_[i]].rectangle_[3].x_ = e;
    map_[graph_vertexes_[i]].rectangle_[3].y_ = n;

    // Set the location of the vertex center
    map_[graph_vertexes_[i]].center_.x_ = (map_[graph_vertexes_[i]].rectangle_[0].x_ + map_[graph_vertexes_[i]].rectangle_[2].x_) / 2.;
    map_[graph_vertexes_[i]].center_.y_ = (map_[graph_vertexes_[i]].rectangle_[0].y_ + map_[graph_vertexes_[i]].rectangle_[2].y_) / 2.;
  }
}

void TopologicalMap::getAdjacentList(vertex_t v, std::vector<uint32_t>* adj_index)
{
  adj_index->clear();
  uint32_t index;
  AdjacencyIterator ai, a_end;
  boost::tie(ai, a_end) = boost::adjacent_vertices(v, map_);

  for (; ai != a_end; ai++)
  {
    index = map_[*ai].index_;
    adj_index->push_back(index);
  }
}

void TopologicalMap::getActiveNodes(const Pose& robot_pose)
{
  // Reset the active nodes
  active_nodes_vertexes_.clear();

  // Go through every vertex
  for (size_t i = 0; i < graph_vertexes_.size(); i++)
  {
    // Compute distance from the robot to the center of the vertex
    Point center = Point(map_[graph_vertexes_[i]].center_.x_, map_[graph_vertexes_[i]].center_.y_, 0.);
    float dist = center.distanceXY(robot_pose.getXYZ());

    // Check if this is an active node
    if (dist <= 30.0)
    {
      active_nodes_vertexes_.push_back(graph_vertexes_[i]);
      active_nodes_ref_[i] = true;
    }
    else
    {
      active_nodes_ref_[i] = false;
    }
  }
}

void TopologicalMap::allocateNodes(const Pose& robot_pose, const std::vector<Point>& points)
{
  // Transform pose to transformation matrix
  std::array<float, 9> l_R;
  robot_pose.toRotMatrix(l_R);
  Tf robot_tf(l_R, std::array<float, 3>{ robot_pose.x_, robot_pose.y_, robot_pose.z_ });

  // Go through every point
  for (const auto& point : points)
  {
    // Transform the point to the map reference frame
    Point tf_point = point * robot_tf;

    // Get the node where the point is located
    vertex_t node;
    if (!getNode(tf_point, node))
    {
      continue;
    }

    // Check if node is already allocated
    if (std::find(allocated_nodes_vertexes_.begin(), allocated_nodes_vertexes_.end(), node) == allocated_nodes_vertexes_.end())
    {
      // Allocate memory either from file, or from the program
      if (map_[node].has_file_)
      {
        // Load file to an occupancy grid map structure
        if (!allocateNodeMapFromFile(node))
        {
          continue;
        }

        // Add node to the allocated nodes array
        allocated_nodes_vertexes_.push_back(node);
      }
      else
      {
        allocateNodeMap(node);
        allocated_nodes_vertexes_.push_back(node);
      }
    }
  }
}

void TopologicalMap::deactivateNodes(const Pose& robot_pose, std::vector<vertex_t>& nodes_to_delete)
{
  // Save and reset allocated nodes array so that we can make a fresh reset
  std::vector<vertex_t> tmp_allocated_nodes_vertexes = allocated_nodes_vertexes_;
  allocated_nodes_vertexes_.clear();

  // Go through every allocated vertex
  for (size_t i = 0; i < tmp_allocated_nodes_vertexes.size(); i++)
  {
    // Compute distance from the robot to the center of the vertex
    Point center = Point(map_[tmp_allocated_nodes_vertexes[i]].center_.x_, map_[tmp_allocated_nodes_vertexes[i]].center_.y_, 0.);
    float dist = center.distanceXY(robot_pose.getXYZ());

    // Check if we want to deallocate this node
    if (dist > 30.0)
    {
      // Save node to delete
      nodes_to_delete.push_back(tmp_allocated_nodes_vertexes[i]);

      // Free the node memory on the localization and mapping interface structure
      localization_mapping_interface_->deallocateNode(map_[tmp_allocated_nodes_vertexes[i]].index_, map_[tmp_allocated_nodes_vertexes[i]].grid_map_);
    }
    else
    {
      allocated_nodes_vertexes_.push_back(tmp_allocated_nodes_vertexes[i]);
    }
  }
}

void TopologicalMap::deallocateNodes(const std::vector<vertex_t>& nodes_to_delete, bool save_to_file)
{
  // Go through every allocated vertex
  for (size_t i = 0; i < nodes_to_delete.size(); i++)
  {
    if (save_to_file)
    {
      // Write the map to the corresponding xml file
      // Create local parameter structure to feed the occupancy grid map
      Parameters l_params;
      l_params.grid_map_files_folder_ = params_.grid_map_files_folder_;
      l_params.gridmap_width_ = map_[nodes_to_delete[i]].grid_map_->width_;
      l_params.gridmap_lenght_ = map_[nodes_to_delete[i]].grid_map_->lenght_;
      l_params.gridmap_height_ = map_[nodes_to_delete[i]].grid_map_->height_;
      l_params.gridmap_origin_x_ = map_[nodes_to_delete[i]].grid_map_->origin_.x_;
      l_params.gridmap_origin_y_ = map_[nodes_to_delete[i]].grid_map_->origin_.y_;
      l_params.gridmap_origin_z_ = map_[nodes_to_delete[i]].grid_map_->origin_.z_;
      l_params.gridmap_resolution_ = map_[nodes_to_delete[i]].grid_map_->resolution_;
      MapWriter mw(l_params, map_[nodes_to_delete[i]].index_);
      mw.writeToFile(map_[nodes_to_delete[i]].grid_map_, l_params);

      // Save the information that this file was already saved into a file
      if (std::find(saved_nodes_.begin(), saved_nodes_.end(), map_[nodes_to_delete[i]].index_) == saved_nodes_.end())
      {
        saved_nodes_.push_back(map_[nodes_to_delete[i]].index_);
      }

      // Set the has_file flag
      map_[nodes_to_delete[i]].has_file_ = true;
    }

    // Free the map memory
    map_[nodes_to_delete[i]].grid_map_->deallocateAllMem();
    delete map_[nodes_to_delete[i]].grid_map_;
    map_[nodes_to_delete[i]].grid_map_ = nullptr;
  }
}

void TopologicalMap::allocateNodeMap(const vertex_t& node)
{
  // Create local parameter structure to feed the occupancy grid map
  Parameters l_params;
  Point center((map_[node].aligned_rectangle_[0].x_ + map_[node].aligned_rectangle_[2].x_) / 2., (map_[node].aligned_rectangle_[0].y_ + map_[node].aligned_rectangle_[2].y_) / 2.);
  float c = 2 * params_.gridmap_resolution_;
  l_params.gridmap_resolution_ = params_.gridmap_resolution_;
  l_params.gridmap_width_ = std::fabs(map_[node].aligned_rectangle_[0].x_ - map_[node].aligned_rectangle_[2].x_) + c;
  l_params.gridmap_lenght_ = std::fabs(map_[node].aligned_rectangle_[0].y_ - map_[node].aligned_rectangle_[2].y_) + c;
  l_params.gridmap_height_ = params_.gridmap_height_;
  l_params.gridmap_origin_x_ = center.x_ - (l_params.gridmap_width_ / 2);
  l_params.gridmap_origin_y_ = center.y_ - (l_params.gridmap_lenght_ / 2);
  l_params.gridmap_origin_z_ = params_.gridmap_origin_z_;

  // Allocate memory for the map
  map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);

  // Replicate the allocation to the localization and mapping interface structure
  localization_mapping_interface_->allocateNode(map_[node].index_, map_[node].center_.x_, map_[node].center_.y_, map_[node].rectangle_orientation_, map_[node].grid_map_);
}

bool TopologicalMap::allocateNodeMapFromFile(const vertex_t& node)
{
  // Load file to an occupancy grid map structure
  Parameters l_params;
  l_params.map_input_file_ = params_.grid_map_files_folder_ + "map_" + std::to_string(map_[node].index_) + ".xml";
  MapParser map_parser(l_params);
  if (!map_parser.parseHeader(&l_params))
  {
#if VERBOSE == 1
    std::cout << "Map input file not found (" << l_params.map_input_file_ << ")." << std::endl;
#endif
    return false;
  }
  else
  {
#if VERBOSE == 1
    std::cout << "Loaded map " << l_params.map_input_file_ << "\n";
#endif
    map_[node].grid_map_ = new OccupancyMap(l_params, Pose(0, 0, 0, 0, 0, 0), 20, 5);
  }

  if (!map_parser.parseFile(&(*map_[node].grid_map_)))
  {
#if VERBOSE == 1
    std::cout << "Map input file not found (" << l_params.map_input_file_ << ")." << std::endl;
#endif
    delete map_[node].grid_map_;
    map_[node].grid_map_ = nullptr;
    return false;
  }

  // If everything went ok, we can initialize the node on the localization and mapping interface structure
  localization_mapping_interface_->allocateNode(map_[node].index_, map_[node].grid_map_->getCorners(), map_[node].grid_map_->getPlanars(), map_[node].center_.x_, map_[node].center_.y_, map_[node].rectangle_orientation_, map_[node].grid_map_);

  return true;
}

bool TopologicalMap::getNodeDeprecated(const Point& point, vertex_t& node)
{
  // Init vars
  float min_dist = std::numeric_limits<float>::max();
  bool found_solution = false;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    // Compute distance from the point to the center of the vertex
    Point center = Point(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0.);
    float dist = center.distanceXY(point);

    // Find the closest node
    if (dist < min_dist)
    {
      if (map_[active_nodes_vertexes_[i]].aligned_rectangle_.size() != 4)
      {
        continue;
      }

      // Check if the point lies inside the node rectangle
      // - transform the point to the aligned rectangles space
      Point aligned_point;
      alignPoint(point, center, map_[active_nodes_vertexes_[i]].rectangle_orientation_, aligned_point);
      // - calculate the distance from the aligned point to the rectangle center component-wise
      float dist_x = std::fabs(aligned_point.x_ - ((map_[active_nodes_vertexes_[i]].aligned_rectangle_[0].x_ + map_[active_nodes_vertexes_[i]].aligned_rectangle_[2].x_) / 2.));
      float dist_y = std::fabs(aligned_point.y_ - ((map_[active_nodes_vertexes_[i]].aligned_rectangle_[0].y_ + map_[active_nodes_vertexes_[i]].aligned_rectangle_[2].y_) / 2.));
      float size_x = std::fabs(map_[active_nodes_vertexes_[i]].aligned_rectangle_[0].x_ - map_[active_nodes_vertexes_[i]].aligned_rectangle_[2].x_);
      float size_y = std::fabs(map_[active_nodes_vertexes_[i]].aligned_rectangle_[0].y_ - map_[active_nodes_vertexes_[i]].aligned_rectangle_[2].y_);

      // We found a possible node for the point
      if (dist_x < (size_x / 2.) && dist_y < (size_y / 2.))
      {
        min_dist = dist;
        node = active_nodes_vertexes_[i];
        found_solution = true;
      }
    }
  }

  return found_solution;
}

bool TopologicalMap::getNode(const Point& point, vertex_t& node)
{
  // Get lookup table indexes
  int i_img = static_cast<int>((point.x_ - look_up_table_min_x_) / look_up_table_res_);
  int j_img = static_cast<int>((point.y_ - look_up_table_min_y_) / look_up_table_res_);

  // Save the node number
  uint16_t node_number = 0;
  if (i_img >= 0 && i_img < look_up_table_img_.cols && j_img >= 0 && j_img < look_up_table_img_.rows)
  {
    node_number = look_up_table_img_.at<ushort>(j_img, i_img);
  }
  else
  {
    return false;
  }

  // Get the node reference
  bool is_active = active_nodes_ref_[node_number - 1];
  node = graph_vertexes_[node_number - 1];

  return (node_number != 0 && is_active);
}

bool TopologicalMap::getCell(const Point& point, Cell& cell, vertex_t& node, bool read_only)
{
  // Get the node corresponding to the input point
  if (!getNode(point, node))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    // if this call is read only, we have nothing to do
    // else, we will allocate the map
    if (read_only)
    {
      return false;
    }
    else
    {
      if (map_[node].has_file_)  // Map is not allocated but is saved on a file
      {
        // Load file to an occupancy grid map structure
        if (!allocateNodeMapFromFile(node))
        {
          return false;
        }

        // Add node to the allocated nodes array
        allocated_nodes_vertexes_.push_back(node);
      }
      else  // Map is not allocated, neither is saved on a file
      {
        // create the occupancy grid map structure
        allocateNodeMap(node);

        // Add node to the allocated nodes array
        allocated_nodes_vertexes_.push_back(node);
      }
    }
  }

  // If we got here, the occupancy grid map was allocated. So, we can access the cell
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(point, center, map_[node].rectangle_orientation_, aligned_point);
  aligned_point.z_ = point.z_;

  // Get cell
  if (map_[node].grid_map_->isInside(aligned_point.x_, aligned_point.y_, aligned_point.z_))
  {
    cell = (*map_[node].grid_map_)(aligned_point.x_, aligned_point.y_, aligned_point.z_);
    return true;
  }
  else
  {
    return false;
  }
}

void TopologicalMap::alignPoint(const Point& in_pt, const Point& reference, const float& angle, Point& out_pt)
{
  float x1 = in_pt.x_ - reference.x_;
  float y1 = in_pt.y_ - reference.y_;

  out_pt.x_ = x1 * std::cos(angle) - y1 * std::sin(angle);
  out_pt.y_ = x1 * std::sin(angle) + y1 * std::cos(angle);

  out_pt.x_ += reference.x_;
  out_pt.y_ += reference.y_;
}

std::vector<Planar> TopologicalMap::getPlanars(const float& x, const float& y, const float& z, bool read_only)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, z);
  std::vector<Planar> planars;

  if (!getCell(pt, c, node, read_only))
  {
    return planars;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get planar features on the cell
  if (c.data != nullptr)
  {
    if (c.data->planar_features_ != nullptr)
    {
      std::vector<Planar> l_planars = *c.data->planar_features_;
      for (auto& ft : l_planars)
      {
        Point l_pt;
        alignPoint(ft.pos_, center, angle, l_pt);
        ft.pos_.x_ = l_pt.x_;
        ft.pos_.y_ = l_pt.y_;

        planars.push_back(ft);
      }
    }
  }

  return planars;
}

std::vector<Corner> TopologicalMap::getCorners(const float& x, const float& y, const float& z, bool read_only)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, z);
  std::vector<Corner> corners;

  // Get the cell
  if (!getCell(pt, c, node, read_only))
  {
    return corners;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get corner features on the cell
  if (c.data != nullptr)
  {
    if (c.data->corner_features_ != nullptr)
    {
      std::vector<Corner> l_corners = *c.data->corner_features_;
      for (auto& ft : l_corners)
      {
        Point l_pt;
        alignPoint(ft.pos_, center, angle, l_pt);
        ft.pos_.x_ = l_pt.x_;
        ft.pos_.y_ = l_pt.y_;

        corners.push_back(ft);
      }
    }
  }

  return corners;
}

std::map<int, SemanticFeature> TopologicalMap::getSemanticFeatures(const float& x, const float& y, bool read_only)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, 0);
  std::map<int, SemanticFeature> landmarks;

  // Get the cell
  if (!getCell(pt, c, node, read_only))
  {
    return landmarks;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get semantic features on the cell
  if (c.data != nullptr)
  {
    if (c.data->landmarks_ != nullptr)
    {
      std::map<int, SemanticFeature> l_landmarks = *c.data->landmarks_;
      for (auto& ft : l_landmarks)
      {
        Point l_pt;
        alignPoint(ft.second.pos_, center, angle, l_pt);
        ft.second.pos_.x_ = l_pt.x_;
        ft.second.pos_.y_ = l_pt.y_;

        landmarks[ft.first] = ft.second;
      }
    }
  }

  return landmarks;
}

std::vector<ImageFeature> TopologicalMap::getImageFeatures(const float& x, const float& y, const float& z, bool read_only)
{
  Cell c;
  vertex_t node;
  Point pt(x, y, z);
  std::vector<ImageFeature> image_features;

  // Get the cell
  if (!getCell(pt, c, node, read_only))
  {
    return image_features;
  }

  // Get rotation parameters
  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Get image features on the cell
  if (c.data != nullptr)
  {
    if (c.data->surf_features_ != nullptr)
    {
      std::vector<ImageFeature> l_image_features = *c.data->surf_features_;
      for (auto& ft : l_image_features)
      {
        Point l_pt;
        alignPoint(ft.pos_, center, angle, l_pt);
        ft.pos_.x_ = l_pt.x_;
        ft.pos_.y_ = l_pt.y_;

        image_features.push_back(ft);
      }
    }
  }

  return image_features;
}

std::vector<Planar> TopologicalMap::getPlanars()
{
  std::vector<Planar> planars;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::vector<Planar> l_planars = map_[active_nodes_vertexes_[i]].grid_map_->getPlanars();
      for (auto& ft : l_planars)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        planars.push_back(ft);
      }
    }
  }

  return planars;
}

std::vector<Corner> TopologicalMap::getCorners()
{
  std::vector<Corner> corners;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::vector<Corner> l_corners = map_[active_nodes_vertexes_[i]].grid_map_->getCorners();
      for (auto& ft : l_corners)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        corners.push_back(ft);
      }
    }
  }

  return corners;
}

std::map<int, SemanticFeature> TopologicalMap::getSemanticFeatures()
{
  std::map<int, SemanticFeature> landmarks;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::map<int, SemanticFeature> l_landmarks = map_[active_nodes_vertexes_[i]].grid_map_->getLandmarks();
      for (auto& ft : l_landmarks)
      {
        Point pt;
        alignPoint(ft.second.pos_, center, angle, pt);
        ft.second.pos_.x_ = pt.x_;
        ft.second.pos_.y_ = pt.y_;

        landmarks[ft.first] = ft.second;
      }
    }
  }

  return landmarks;
}

std::vector<ImageFeature> TopologicalMap::getImageFeatures()
{
  std::vector<ImageFeature> images_features;

  // Go through every active vertex
  for (size_t i = 0; i < active_nodes_vertexes_.size(); i++)
  {
    Point center(map_[active_nodes_vertexes_[i]].center_.x_, map_[active_nodes_vertexes_[i]].center_.y_, 0);
    float angle = -map_[active_nodes_vertexes_[i]].rectangle_orientation_;

    if (map_[active_nodes_vertexes_[i]].grid_map_ != nullptr)
    {
      std::vector<ImageFeature> l_image_features = map_[active_nodes_vertexes_[i]].grid_map_->getImageFeatures();
      for (auto& ft : l_image_features)
      {
        Point pt;
        alignPoint(ft.pos_, center, angle, pt);
        ft.pos_.x_ = pt.x_;
        ft.pos_.y_ = pt.y_;

        images_features.push_back(ft);
      }
    }
  }

  return images_features;
}

bool TopologicalMap::insert(const Planar& planar)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(planar.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      if (!allocateNodeMapFromFile(node))
      {
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(planar.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Planar transformed_ft = planar;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  bool success = map_[node].grid_map_->insert(transformed_ft);

  // Update the localization and mapping interface structure
  if (success)
  {
    localization_mapping_interface_->updateCell(map_[node].index_, static_cast<uint8_t>(1), transformed_ft, map_[node].grid_map_);
  }

  return true;
}

bool TopologicalMap::insert(const Corner& corner)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(corner.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      if (!allocateNodeMapFromFile(node))
      {
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(corner.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Corner transformed_ft = corner;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  int success = map_[node].grid_map_->insert(transformed_ft);

  // Update the localization and mapping interface structure
  if (success)
  {
    localization_mapping_interface_->updateCell(map_[node].index_, static_cast<uint8_t>(1), transformed_ft, map_[node].grid_map_);
  }

  return true;
}

bool TopologicalMap::insert(const SemanticFeature& landmark, const int& id)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(landmark.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      if (!allocateNodeMapFromFile(node))
      {
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(landmark.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  SemanticFeature transformed_ft = landmark;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->insert(transformed_ft, id);

  // Update the localization and mapping interface structure
  //  localization_mapping_interface_->updateCell(map_[node].index_, static_cast<uint8_t>(1), transformed_ft.pos_.x_,
  //                                              transformed_ft.pos_.y_, transformed_ft.pos_.z_, map_[node].grid_map_);

  return true;
}

bool TopologicalMap::insert(const ImageFeature& image_feature)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(image_feature.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      if (!allocateNodeMapFromFile(node))
      {
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated, neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(image_feature.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  ImageFeature transformed_ft = image_feature;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  map_[node].grid_map_->insert(transformed_ft);

  // Update the localization and mapping interface structure
  //  localization_mapping_interface_->updateCell(map_[node].index_, static_cast<uint8_t>(1), transformed_ft.pos_.x_,
  //                                              transformed_ft.pos_.y_, transformed_ft.pos_.z_, map_[node].grid_map_);

  return true;
}

bool TopologicalMap::directInsert(const Planar& planar)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(planar.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      if (!allocateNodeMapFromFile(node))
      {
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated, neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(planar.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Planar transformed_ft = planar;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  bool success = map_[node].grid_map_->directInsert(transformed_ft);

  // Update the localization and mapping interface structure
  if (success)
  {
    localization_mapping_interface_->updateCell(map_[node].index_, static_cast<uint8_t>(1), transformed_ft, map_[node].grid_map_);
  }

  return true;
}

bool TopologicalMap::directInsert(const Corner& corner)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(corner.pos_, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[node].grid_map_ == nullptr)
  {
    if (map_[node].has_file_)  // Map is not allocated but is saved on a file
    {
      // Load file to an occupancy grid map structure
      if (!allocateNodeMapFromFile(node))
      {
        return false;
      }

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
    else  // Map is not allocated, neither is saved on a file
    {
      // create the occupancy grid map structure
      allocateNodeMap(node);

      // Add node to the allocated nodes array
      allocated_nodes_vertexes_.push_back(node);
    }
  }

  // If we got here, the occupancy grid map was allocated
  // So, we will transform the feature
  Point aligned_point, center(map_[node].center_.x_, map_[node].center_.y_, 0);
  alignPoint(corner.pos_, center, map_[node].rectangle_orientation_, aligned_point);
  Corner transformed_ft = corner;
  transformed_ft.pos_.x_ = aligned_point.x_;
  transformed_ft.pos_.y_ = aligned_point.y_;

  // And now we store it
  int success = map_[node].grid_map_->directInsert(transformed_ft);

  // Update the localization and mapping interface structure
  if (success)
  {
    localization_mapping_interface_->updateCell(map_[node].index_, static_cast<uint8_t>(1), transformed_ft, map_[node].grid_map_);
  }

  return true;
}

bool TopologicalMap::update(const Planar& old_planar, const Planar& new_planar)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_planar.pos_, old_node);
  bool get_new_node = getNode(new_planar.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[old_node].center_.x_, map_[old_node].center_.y_, 0);
    alignPoint(old_planar.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_planar.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    Planar transformed_old_ft = old_planar;
    Planar transformed_new_ft = new_planar;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    bool a = map_[new_node].grid_map_->update(transformed_old_ft, transformed_new_ft);

    // Update localization and mapping interface structure
    if (a)
    {
      // Check:
      // - only if the cell holding the old feature got empty
      if ((*map_[old_node].grid_map_)(transformed_old_ft.pos_.x_, transformed_old_ft.pos_.y_, transformed_old_ft.pos_.z_).data->planar_features_->size() == 0)
      {
        localization_mapping_interface_->updateCell(map_[old_node].index_, static_cast<uint8_t>(0), transformed_old_ft, map_[old_node].grid_map_);
      }
      if ((*map_[new_node].grid_map_)(transformed_new_ft.pos_.x_, transformed_new_ft.pos_.y_, transformed_new_ft.pos_.z_).data->planar_features_->size() > 0)
      {
        localization_mapping_interface_->updateCell(map_[new_node].index_, static_cast<uint8_t>(1), transformed_new_ft, map_[new_node].grid_map_);
      }
    }

    return a;
  }
}

bool TopologicalMap::update(const Corner& old_corner, const Corner& new_corner)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_corner.pos_, old_node);
  bool get_new_node = getNode(new_corner.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[new_node].center_.x_, map_[new_node].center_.y_, 0);
    alignPoint(old_corner.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_corner.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    Corner transformed_old_ft = old_corner;
    Corner transformed_new_ft = new_corner;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    bool a = map_[new_node].grid_map_->update(transformed_old_ft, transformed_new_ft);

    // Update localization and mapping interface structure
    if (a)
    {
      // Check:
      // - only if the cell holding the old feature got empty
      if ((*map_[old_node].grid_map_)(transformed_old_ft.pos_.x_, transformed_old_ft.pos_.y_, transformed_old_ft.pos_.z_).data->corner_features_->size() == 0)
      {
        localization_mapping_interface_->updateCell(map_[old_node].index_, static_cast<uint8_t>(0), transformed_old_ft, map_[old_node].grid_map_);
      }
      if ((*map_[new_node].grid_map_)(transformed_new_ft.pos_.x_, transformed_new_ft.pos_.y_, transformed_new_ft.pos_.z_).data->corner_features_->size() > 0)
      {
        localization_mapping_interface_->updateCell(map_[new_node].index_, static_cast<uint8_t>(1), transformed_new_ft, map_[new_node].grid_map_);
      }
    }

    return a;
  }
}

bool TopologicalMap::update(const SemanticFeature& new_landmark, const SemanticFeature& old_landmark, const int& old_landmark_id)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_landmark.pos_, old_node);
  bool get_new_node = getNode(new_landmark.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[new_node].center_.x_, map_[new_node].center_.y_, 0);
    alignPoint(old_landmark.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_landmark.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    SemanticFeature transformed_old_ft = old_landmark;
    SemanticFeature transformed_new_ft = old_landmark;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    return map_[new_node].grid_map_->update(transformed_new_ft, transformed_old_ft, old_landmark_id);
  }
}

bool TopologicalMap::update(const ImageFeature& old_image_feature, const ImageFeature& new_image_feature)
{
  // Get the nodes corresponding to the input features
  vertex_t old_node, new_node;
  bool get_old_node = getNode(old_image_feature.pos_, old_node);
  bool get_new_node = getNode(new_image_feature.pos_, new_node);

  // Check if the nodes are active
  if (!get_old_node || !get_new_node || (map_[old_node].index_ != map_[new_node].index_))
  {
    return false;
  }

  // Check if the OccupancyMap of the node is allocated
  if (map_[new_node].grid_map_ == nullptr)
  {
    return false;
  }
  else
  {
    // Transform the features
    Point aligned_old_point, aligned_new_point, center(map_[new_node].center_.x_, map_[new_node].center_.y_, 0);
    alignPoint(old_image_feature.pos_, center, map_[new_node].rectangle_orientation_, aligned_old_point);
    alignPoint(new_image_feature.pos_, center, map_[new_node].rectangle_orientation_, aligned_new_point);
    ImageFeature transformed_old_ft = old_image_feature;
    ImageFeature transformed_new_ft = new_image_feature;
    transformed_old_ft.pos_.x_ = aligned_old_point.x_;
    transformed_old_ft.pos_.y_ = aligned_old_point.y_;
    transformed_new_ft.pos_.x_ = aligned_new_point.x_;
    transformed_new_ft.pos_.y_ = aligned_new_point.y_;

    // Update the features
    return map_[new_node].grid_map_->update(transformed_old_ft, transformed_new_ft);
  }
}

void TopologicalMap::downsamplePlanars()
{
  // Go through every allocated vertex
  for (size_t i = 0; i < allocated_nodes_vertexes_.size(); i++)
  {
    // Downsample each grid map
    map_[allocated_nodes_vertexes_[i]].grid_map_->downsamplePlanars();
  }
}

void TopologicalMap::downsampleCorners()
{
  // Go through every allocated vertex
  for (size_t i = 0; i < allocated_nodes_vertexes_.size(); i++)
  {
    // Downsample each grid map
    map_[allocated_nodes_vertexes_[i]].grid_map_->downsampleCorners();
  }
}

bool TopologicalMap::getAdjacent(const float& x, const float& y, const float& z, const int& layers, vertex_t& node, std::vector<Cell>& adjacent)
{
  // Get the node corresponding to the input point
  Point pt(x, y, z);
  bool get_node = getNode(pt, node);

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }
  else
  {
    // Transform the input point to the local grid map
    Point pt_tf;
    Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
    double angle = map_[node].rectangle_orientation_;
    alignPoint(pt, center, angle, pt_tf);

    // Call the getAdjacent function of OccupancyMap
    return map_[node].grid_map_->getAdjacent(pt_tf.x_, pt_tf.y_, pt_tf.z_, layers, adjacent);
  }
}

bool TopologicalMap::getAdjacentFeatures(const float& x, const float& y, const float& z, const int& layers, std::map<int, SemanticFeature>& features)
{
  // Get adjacent cells
  vertex_t node;
  std::vector<Cell> adjacent;
  if (!getAdjacent(x, y, z, layers, node, adjacent))
  {
    return false;
  }

  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = -map_[node].rectangle_orientation_;

  // Go through every cell
  for (const auto& cell : adjacent)
  {
    if (cell.data != nullptr)
    {
      if (cell.data->landmarks_ != nullptr)
      {
        for (const auto& landmark : *cell.data->landmarks_)
        {
          // Transform the features to the global map
          SemanticFeature l_landmark = landmark.second;
          Point aligned_pt;
          alignPoint(l_landmark.pos_, center, angle, aligned_pt);
          l_landmark.pos_.x_ = aligned_pt.x_;
          l_landmark.pos_.y_ = aligned_pt.y_;

          // Save them
          features[landmark.first] = l_landmark;
        }
      }
    }
  }

  return true;
}

bool TopologicalMap::findNearest(const Planar& input, Planar& nearest, float& sdist)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(input.pos_, node);

  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = map_[node].rectangle_orientation_;

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }
  else
  {
    // Transform the feature to the local map
    Point aligned_pt;
    alignPoint(input.pos_, center, angle, aligned_pt);
    Planar l_input, l_nearest;
    l_input.pos_.x_ = aligned_pt.x_;
    l_input.pos_.y_ = aligned_pt.y_;
    l_input.pos_.z_ = input.pos_.z_;

    // Find the nearest feature
    if (!map_[node].grid_map_->findNearest(l_input, l_nearest, sdist))
    {
      return false;
    }
    else
    {
      // Transform the feature back to the global map
      alignPoint(l_nearest.pos_, center, -angle, nearest.pos_);
    }
  }

  return true;
}

bool TopologicalMap::findNearest(const Corner& input, Corner& nearest, float& sdist)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(input.pos_, node);

  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = map_[node].rectangle_orientation_;

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }
  else
  {
    // Transform the feature to the local map
    Point aligned_pt;
    alignPoint(input.pos_, center, angle, aligned_pt);
    Corner l_input, l_nearest;
    l_input.pos_.x_ = aligned_pt.x_;
    l_input.pos_.y_ = aligned_pt.y_;
    l_input.pos_.z_ = input.pos_.z_;

    // Find the nearest feature
    if (!map_[node].grid_map_->findNearest(l_input, l_nearest, sdist))
    {
      return false;
    }
    else
    {
      // Transform the feature back to the global map
      alignPoint(l_nearest.pos_, center, -angle, nearest.pos_);
    }
  }

  return true;
}

bool TopologicalMap::findNearest(const ImageFeature& input, ImageFeature& nearest, float& ddist)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(input.pos_, node);

  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = map_[node].rectangle_orientation_;

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }
  else
  {
    // Transform the feature to the local map
    Point aligned_pt;
    alignPoint(input.pos_, center, angle, aligned_pt);
    ImageFeature l_input, l_nearest;
    l_input.pos_.x_ = aligned_pt.x_;
    l_input.pos_.y_ = aligned_pt.y_;
    l_input.pos_.z_ = input.pos_.z_;

    // Find the nearest feature
    if (!map_[node].grid_map_->findNearest(l_input, l_nearest, ddist))
    {
      return false;
    }
    else
    {
      // Transform the feature back to the global map
      alignPoint(l_nearest.pos_, center, -angle, nearest.pos_);
    }
  }

  return true;
}

bool TopologicalMap::findNearestOnCell(const ImageFeature& input, ImageFeature& nearest)
{
  // Get the node corresponding to the input point
  vertex_t node;
  bool get_node = getNode(input.pos_, node);

  Point center(map_[node].center_.x_, map_[node].center_.y_, 0);
  double angle = map_[node].rectangle_orientation_;

  // Check if the node is active
  if (!get_node)
  {
    return false;
  }
  else
  {
    // Transform the feature to the local map
    Point aligned_pt;
    alignPoint(input.pos_, center, angle, aligned_pt);
    ImageFeature l_input, l_nearest;
    l_input.pos_.x_ = aligned_pt.x_;
    l_input.pos_.y_ = aligned_pt.y_;
    l_input.pos_.z_ = input.pos_.z_;

    // Find the nearest feature
    if (!map_[node].grid_map_->findNearestOnCell(l_input, l_nearest))
    {
      return false;
    }
    else
    {
      // Transform the feature back to the global map
      alignPoint(l_nearest.pos_, center, -angle, nearest.pos_);
    }
  }

  return true;
}

}  // namespace vineslam