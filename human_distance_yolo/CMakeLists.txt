cmake_minimum_required(VERSION 3.8)
project(human_distance_yolo)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_package(navigation_msgs REQUIRED)

# Directory used to install the package Python dependencies during the build
set(PY_DEPS_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/python_deps)


# Install requirements.txt into the local vendor directory during colcon build 
add_custom_target(install_pydeps ALL
  COMMAND ${Python3_EXECUTABLE} -m pip install --upgrade pip
  COMMAND ${Python3_EXECUTABLE} -m pip install --no-cache-dir 
                                -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
                                --target=${PY_DEPS_BUILD_DIR}
  COMMENT "Installing local Python dependencies into the vendor directory: ${PY_DEPS_BUILD_DIR}"                              
)

# Copy the vendor dependencies to the install folder 
install(DIRECTORY ${PY_DEPS_BUILD_DIR}/
  DESTINATION lib/${PROJECT_NAME}/python_deps)
  

# Generate an environment hook to add the vendored Python deps to PYTHONPATH when ROS2 ws is sourced
set(PYTHONPATH_DSV ${CMAKE_CURRENT_BINARY_DIR}/pythonpath.dsv)

file(WRITE ${PYTHONPATH_DSV}
"prepend-non-duplicate;PYTHONPATH;${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/python_deps\n")

ament_environment_hooks(${PYTHONPATH_DSV})


# Install the Python script as a ROS2 executable
install(PROGRAMS
  src/human_distance_yolo.py  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
