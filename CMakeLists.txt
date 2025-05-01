# Definir política CMP0148 para evitar warnings relacionados ao Python
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

# Versão mínima do CMake
cmake_minimum_required(VERSION 3.10)

# Nome do projeto
project(md25_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Incluir diretório de cabeçalhos
include_directories(include)

# Definir arquivos de mensagem
set(msg_files
  "msg/Md25Data.msg"
  "msg/Md25Encoders.msg"
)

# Gerar interfaces ROS
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
)

# Adicionar executável para o nó motor_controller_node
add_executable(motor_controller_node 
               src/motor_controller/motor_controller_node.cpp 
               src/motor_controller/motor_controller.cpp
               src/motor_controller/PID_control.cpp)

# Dependências do executável motor_controller_node
ament_target_dependencies(motor_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
  rosidl_typesupport_c
)

# Linkar bibliotecas geradas para suporte de tipos (motor_controller_node)
target_link_libraries(motor_controller_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_c
)

# Adicionar executável para o nó odometry_node
add_executable(odometry_node 
               src/odometry/odometry_node.cpp
               src/odometry/odometry.cpp)

# Dependências do executável odometry_node
ament_target_dependencies(odometry_node
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
  rosidl_typesupport_c
)

# Linkar bibliotecas geradas para suporte de tipos (odometry_node)
target_link_libraries(odometry_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_c
)

# Adicionar executável para o nó md25_base_controller_node
add_executable(md25_base_controller_node 
               src/md25_base_controller/md25_base_controller_node.cpp 
               src/md25_base_controller/md25_base_controller.cpp
               src/md25_base_controller/MD25Driver/md25_driver.cpp)

# Dependências do executável md25_base_controller_node
ament_target_dependencies(md25_base_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
  rosidl_typesupport_c
)

# Linkar bibliotecas geradas para suporte de tipos (md25_base_controller_node)
target_link_libraries(md25_base_controller_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_c
)

# Instalar os executáveis
install(TARGETS
  motor_controller_node 
  odometry_node
  md25_base_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Instalar diretórios adicionais (launch, config)
install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME})

# Exportar dependências
ament_export_dependencies(rosidl_default_runtime)

# Exportar o pacote
ament_package()