cmake_minimum_required(VERSION 3.5)
project(motion_model_debugger)

# Encontrar as dependências necessárias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vineslam REQUIRED)

# Adicionar o executável
add_executable(motion_model_debugger src/motion_model_debugger.cpp)

# Linkar as bibliotecas necessárias
ament_target_dependencies(motion_model_debugger
        rclcpp
        tf2
        tf2_ros
        geometry_msgs
        tf2_geometry_msgs
        vineslam
        )

# Instalação do executável
install(TARGETS motion_model_debugger
        DESTINATION lib/${PROJECT_NAME}
        )

# Instalação do diretório launch, caso você queira criar um arquivo de launch posteriormente
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

# Configuração do ament
ament_package()