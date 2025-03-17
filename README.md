<div align="center">
  <p align="center">
    <img src="./docs/INESCTEC_MAIN.png" alt="Logotipo Instituição A" width="200"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
    <img src="./docs/TRIBE_MAIN.png" alt="Logotipo Instituição B" width="200"/>
  </p>
</div>

# VineSLAM

**A multi-layer and multi-sensor Localization and Mapping approach for agricultural environments.**

**VineSLAM** is a localization and mapping algorithm designed for challenging agricultural environments such as mountain vineyards and flat orchards.
This algorithm is based on two main performance optimizers: **(1)** topological mapping to manage the memory resources and enable large-scale and long-term operation; and **(2)** CUDA-based GPU optimizations to improve runtime performance.

This algorithm supports two operation modes: **SLAM**, where a multi-layer map is built while the robot is simultaneously localized; and **localization-only**, using a pre-built map to localize the robot.

This implementation is currently supported in **ROS2 Foxy** and **ROS2 Eloquent**.

<div align="center">
<img align="center" width="600" height="400" src="./docs/vineslam_aosta_seq_robosense_speed.gif" alt=""> 
</div>

---

## Table of Contents
- [Version History](#version-history)
- [Prototype Status](#prototype-status)
- [Software Roadmap](#roadmap)
- [Dependencies](#dependencies)
- [Installation Instructions](#installation-instructions)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributors](#contributors)
- [Acknowledgments](#acknowledgments)
- [License](#license)

---

## Version History
Below is a list of versions released for this project. Each version has a link to its release notes or relevant documentation.

| Version                        | Release Date | Description                                                                                                                                                                                                                     |
|--------------------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [v1.0.0](https://gitlab.inesctec.pt/agrob/prototypes/vineslam/-/tags) | 2024-11-11   | &bull; Fully compatible with ROS2 foxy <br> &bull; Sensors supported: IMU, GNSS, 3D LiDAR, 2D Odometry <br> &bull; Validation in ROSBAG2 datasets: fully tested and validated in indoor/outdoor environments <br> &bull; Deployment on robots in real time: fully tested and validated in indoor environments |

---

## Prototype Status

* In Development: aligned with [Roadmap](#roadmap).
* In Testing Phase: being fully tested in indoor/outdoor urban environments.

---

## Sofware Roadmap
The planned milestones and tasks for the project's development. Completed tasks are marked.

- [x] **ROS2 Compatibility**
    - [x] Compatible with ROS2 foxy
    - [x] Compatible with ROS2 humble
- [x] **Fully working on indoor/outdoor 3D urban environmets**
    - [x] Precise mapping in indoor/outdoor 3D urban environments
    - [x] Precise 3D localization on the robot in real-time in indoor/outdoor 3D urban environments - **extensive testing**
- [ ] **Fully working on outdoor 3D complex agricultural environments**
    - [x] Precise mapping in outdoor 3D complex agricultural environments
    - [x] Short-term tests (<500m) on the robot in real-time in outdoor 3D complex agricultural environments
    - [ ] Long-term tests (>500m) on the robot in real-time in outdoor 3D complex agricultural environments
- [ ] **Particle Filter (PF) improvement**
    - [x] Correct model state to support **pure rotations** misaligned with robot's centre of mass
    - [ ] Improve PF performance on harsh inclinations (roll and pitch)
- [ ] **Semantic Data**
    - [ ] Create mapping process for semantic data in agriculture: trunks, fruits, leaves, and others
    - [ ] Create filtering process for dynamic data: persons, animals, cars, and others
    - [ ] Use semantic mapping process in the localization loop
- [ ] **Integrate Loop Closure**
    - [ ] Explore open-source Loop Closure frameworks
    - [ ] Integrate Loop Closure on VineSLAM using the Topological Map concept
    - [ ] Validate improvements of the mapping process with this implementation

---

## Dependencies

- **Software Dependencies**: OpenCV, Boost, Asio, ros-foxy-full, ros-foxy-diagnostic-updater, ros-foxy-vision-msgs, ros-foxy-ublox-msgs

To install them, run the following commands:
```bash
# General dependencies
sudo apt-get install libopencv-dev
sudo apt-get install libboost-dev
sudo apt-get install libasio-dev

# ROS dependencies (considering a ros-foxy-full installation)
sudo apt-get install ros-foxy-diagnostic-updater
sudo apt-get install ros-foxy-vision-msgs
sudo apt-get install ros-foxy-ublox-msgs
```

- **Optional Dependencies**: To use the GPU optimized version, you must **install CUDA**, from [https://developer.nvidia.com/cuda-downloads](https://developer.nvidia.com/cuda-downloads).

---

## Installation Instructions

### Native Installation
Follow these steps to set up the project in your local environment.

1. **Clone the repository**:
    ```bash
    cd <path_to_ros2_ws>/src
    git clone https://gitlab.inesctec.pt/agrob/vineslam_stack/vineslam -b master
    ```
2. **Compile VineSLAM**:
    ```bash
    cd <path_to_ros2_ws>
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DLIDAR_TYPE=0
    ```
   
The `-DLIDAR_TYPE` cmake argument compiles VineSLAM for specific LiDAR models:
- `-DLIDAR_TYPE=0` corresponds to Velodyne VLP16
- `-DLIDAR_TYPE=1` corresponds to RoboSense Helios 5515
- `-DLIDAR_TYPE=2` corresponds to Livox Mid 70
- `-DLIDAR_TYPE=3` corresponds to Ouster-OS-1-64
- `-DLIDAR_TYPE=4` uses all the features of the subscribed point cloud
- The default value is 0, if no `-DLIDAR_TYPE` is chosen

Note that if you have CUDA installed, VineSLAM will automatically build CUDA kernels and use GPU optimizations.

### Docker Installation
To install and build VineSLAM you can also use the provided [Dockerfile](./docker/Dockerfile).
Note that the Docker environment does not consider CUDA installation.

````bash
git clone https://gitlab.inesctec.pt/agrob/vineslam_stack/vineslam -b master
cd vineslam/docker
docker build -t vineslam .
````

This will create a docker container with VineSLAM ready to compile and execute. Now you just have to...
````bash
docker run -it vineslam bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
````

---

## Usage
Before executing VineSLAM, please check the [Interfaces](./docs/interfaces.md) file. In this you will find the published/subscribed topics, the /tf listened and broadcasted, and the parameters and their meaning.
With this, you can better tune and use VineSLAM.

After performing the **topic remappings** and **parameters tuning** you can simply execute, for example the SLAM Node, as follows:

````bash
cd <path_to_ros2_ws>/src/vineslam/test/test_slam_node
ros2 launch run.launch.py
````

This will automatically open a rviz visualization panel properly configured.

The same can be done for the localization node in the respective test folder.

---

## Project Structure

| Folder/File     | Description                                                                                                                           |
|-----------------|---------------------------------------------------------------------------------------------------------------------------------------|
| `/docker`       | Contains the docker files needed to build vineslam in a container                                                                     |
| `/docs`         | Contains the images used in this README file and the ROS2 interfaces used by VineSLAM                                                 |
| `/test`         | Contains two test folders to easily execute VineSLAM in SLAM and Localization modes and an utilities folder that contains debug nodes |
| `/vineslam`     | Implements all the VineSLAM algorithms (mapping, localization, feature extraction, and others)                                        |
| `/vineslam_ros` | Implements all the VineSLAM ROS2 interfaces (publishers, subscribers, services)                                                       |

---

## Contributors
We thank the following team members for their contributions to this project:

| Name                    | Email                                                           |
|-------------------------|-----------------------------------------------------------------|
| **André Silva Aguiar**  | [andre.s.aguiar@inesctec.pt](mailto:andre.s.aguiar@inesctec.pt) |
| Filipe Neves dos Santos | [fbsantos@inesctec.pt](mailto:fbsantos@inesctec.pt)             |

---

## Acknowledgments
This prototype was funded and developed under the following projects:

- [SCORPION](https://scorpion-h2020.eu/) - SCORPION’s objective is to develop a safe and autonomous precision spraying tool integrated into a modular unmanned tractor (robotics platform) to increase spraying efficiency, while reducing human and animal exposure to pesticides, water usage and labour costs.
- [ROMOVI](https://www.inesctec.pt/en/projects/romovi#about) - The RoMoVi project main objective is the development of robotic components and a modular and extensible mobile platform, which will allow in the future to provide commercial solutions for hillside vineyards capable of autonomously executing operations of monitoring and logistics.
- [ROSIN](https://www.rosin-project.eu/) - ROS-Industrial aims to transfer value and the ease of application to industrial hardware, by developing new components, improving existing ones, but also by performing non-development work such as compiling usage and development guidelines or performing license audits.

<div align="center">
    <p align="center">
      <img src="https://scorpion-h2020.eu/wp-content/uploads/2021/03/logo-scorpion-simple.png" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://www.advid.pt/uploads/PROJETOS/projetos-logos/L7-P1-ROMOVI-Logo_01.png" width="150" style="margin-right: 50px;"/>
    </p>
</div>

<div align="center">
    <p align="center">
      <img src="https://repositorio.inesctec.pt/logos/compete.png" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://repositorio.inesctec.pt/logos/norte2020/portugal2020.svg" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://repositorio.inesctec.pt/logos/ue-feder_cor.jpg" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://repositorio.inesctec.pt/logos/logo_cores.jpg" width="150" style="margin-right: 50px;"/>
    </p>
</div>

---

## License
This project is licensed under the **GNU General Public License v3.0** - see the [LICENSE](LICENSE) file for details.

---
