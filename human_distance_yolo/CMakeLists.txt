cmake_minimum_required(VERSION 3.8)
project(human_distance_yolo)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Carpeta para instalar las deps del pkg (en build/)
set(PY_DEPS_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/python_deps)
#set(PY_DEPS_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/python_deps)

# Instalar requirements.txt en la carpeta anterior cuando colcon build
add_custom_target(install_pydeps ALL
  COMMAND ${Python3_EXECUTABLE} -m pip install --upgrade pip
  COMMAND ${Python3_EXECUTABLE} -m pip install --no-cache-dir 
                                -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
                                --target=${PY_DEPS_BUILD_DIR}
  COMMENT "Instalando dependencias Python locales en el vendor ${PY_DEPS_BUILD_DIR}"                              
)

# Copiar las deps al install 
install(DIRECTORY ${PY_DEPS_BUILD_DIR}/
  DESTINATION lib/${PROJECT_NAME}/python_deps)
  


set(PYTHONPATH_DSV ${CMAKE_CURRENT_BINARY_DIR}/pythonpath.dsv)

file(WRITE ${PYTHONPATH_DSV}
"prepend-non-duplicate;PYTHONPATH;${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/python_deps\n")
ament_environment_hooks(${PYTHONPATH_DSV})


# Para añadir las deps del vendor al PYTHONPATH en runtime
#file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/pythonpath.dsv
#  "PYTHONPATH append ${PY_DEPS_INSTALL_DIR}\n")
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/pythonpath.dsv
#  DESTINATION share/${PROJECT_NAME}/environment)




# Instalar el script py como ejecutable para ROS2
install(PROGRAMS
  src/human_distance_yolo.py  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
