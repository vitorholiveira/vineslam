# VineSLAM ROS architecture

---

### ROS nodes

* `slam_node` - Simultaneous Localization and Mapping node
* `localization_node` - Localization-only node. Loads a map at the initialization.

----

### Subscribed topics

* `/scan_topic`
  ([sensor_msgs::msg::PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)) - LiDAR data
* `/odom_topic`
  ([nav_msgs::msg::Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)) - wheel Odometry
* `/gps_topic`
  ([sensor_msgs::msg::NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)) - GPS topic (optional)
* `/gps_heading_topic`
  ([ublox_msgs::msg::NAVRELPOSNED9](https://github.com/KumarRobotics/ublox/blob/foxy-devel/ublox_msgs/msg/NavRELPOSNED9.msg)) - GPS heading topic (optional)
* `/imu_topic`
  ([geometry_msgs::msg::Vector3Stamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Vector3Stamped.html)) - IMU 3-axis orientation topic (optional)
* `/imu_data_topic`
  ([sensor_msgs::msg::Imu](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)) - IMU 3-axis angular velocity topic (optional)

Note that you can easily **remap** you sensor topics through the launch file.

See [this file](../test/test_slam_node/run.launch.py) as example.

----

### Published topics (relevant-only)

* `/vineslam/pose`
  ([geometry_msgs::msg::PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html)) - the 6-DoF robot pose
* `/vineslam/poses`
  ([geometry_msgs::msg::PoseArray](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseArray.html)) - the 6-DoF particles pose
* `/vineslam/topological_map`
  ([visualization_msgs::MarkerArray](https://docs.ros2.org/foxy/api/visualization_msgs/msg/MarkerArray.html)) - the topological map graphical representation
* `/vineslam/map3D/corners_map`
  ([sensor_msgs::msg::PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)) - the 3D PointCloud corner feature global map
* `/vineslam/map3D/planars_map`
  ([sensor_msgs:::msg::PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)) - the 3D PointCloud planar features global map
* `/vineslam/map3D/planes_map`
  ([visualization_msgs::msg::MarkerArray](https://docs.ros2.org/foxy/api/visualization_msgs/msg/MarkerArray.html)) - the 3D PointCloud plane features global map
* `/vineslam/map3D/corners_local`
  ([sensor_msgs::msg::PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)) - the 3D PointCloud corner feature local map
* `/vineslam/map3D/planars_local`
  ([sensor_msgs::msg::PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)) - the 3D PointCloud planar features local map
* `/vineslam/map3D/planes_local`
  ([visualization_msgs::msg::MarkerArray](https://docs.ros2.org/foxy/api/visualization_msgs/msg/MarkerArray.html)) - the 3D PointCloud plane features local map

----

### Tf broadcasters

* `/odom` to `global_frame_id`.

### Tf listeners

* `base_frame_id` to `lidar_sensor_frame`.
* `base_frame_id` to `imu_sensor_frame` (if used)

----

### Parameters

* `world_frame_id`: ROS frame id of the map
* `base_frame_id`: ROS frame id of the robot's base
* `lidar_sensor_frame`: ROS frame id of the LiDAR used for localization and mapping
* `imu_sensor_frame`: ROS frame id of the IMU used (if used)


* `use_lidar_features` - whether to use or not the 3D LiDAR features
* `use_vertical_planes` - whether to use or not the 3D LiDAR vertical plane features
* `use_ground_plane` - whether to use or not the 3D LiDAR ground plane feature
* `use_imu` - whether to use or not the IMU sensor
* `use_gyroscope` - whether to use or not the gyro sensor
* `use_gps` - whether to use or not the GPS LAT/LON components
* `use_gps_altitude` - whether to use or not the GPS ALT component
* `use_gps_heading` - whether to use or not the GPS HEADING component


* `publish_level` - 0 - do not publish any map; 1 - publish only the 3D map; 2 - publish all the maps


* `robot_dimensions/[x,y,z]` - robot dimensions used to filter the LiDAR points lying on the robot's structure


* `multilayer-mapping`:
    * `datum`:
        * `latitude`: map's origin on latitude component
        * `longitude`: map's origin on longitude component
        * `altitude`: map's origin on altitude component
    * `topological_map`:
        * `autogen_topological_map`: whether to use a pre-built map, or to automatically generate one on runtime
        * `folder`: directory where the maps and all the information will be stored
        * `dimensions`:
            * `x`: x-size of the topological map
            * `y`: y-size of the topological map
            * `square_size`: square size of each topological node
    * `grid_map`:
        * `origin`:
            * `z` - z coordinate of grid map origin
        * `height` - map height (meters)
        * `resolution` - map resolution (meters)
* `pf`:
    * `n_particles` - number of particles of the particle filter
    * `sigma_xx` - motion model xx constant (meters)
    * `sigma_yy` - motion model yy constant (meters)
    * `sigma_zz` - motion model zz constant (meters)
    * `sigma_RR` - motion model RR constant (radians)
    * `sigma_PP` - motion model PP constant (radians)
    * `sigma_RR` - motion model YY constant (radians)