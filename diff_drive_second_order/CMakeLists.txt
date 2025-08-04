cmake_minimum_required(VERSION 3.8)
project(diff_drive_second_order)


# Buscar paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(ignition-common4 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(Protobuf REQUIRED)

# Obtener los números de versión
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

# Añade la librería compartida
add_library(${PROJECT_NAME} SHARED
  src/DiffDrive_second_order.cc
)

# Requiere C++17
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

# Añade los includes
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${ignition-gazebo6_INCLUDE_DIRS}
    ${ignition-plugin1_INCLUDE_DIRS}
    ${ignition-common4_INCLUDE_DIRS}
    ${ignition-transport11_INCLUDE_DIRS}
)

# Enlaza las librerías necesarias
target_link_libraries(${PROJECT_NAME}
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  protobuf::libprotobuf
)

# Instalar
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)


# Finalizar con ament
ament_package()