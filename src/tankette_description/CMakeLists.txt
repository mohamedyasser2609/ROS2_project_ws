cmake_minimum_required(VERSION 3.8)
project(tankette_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the get_default_rmw_implementation() call below if you know ahead of time
# which rmw_implementation you want to use. Otherwise, it will use the default
# rmw_implementation set by the user in the environment.
# find_package(rmw_implementation REQUIRED)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a source build)
  # comment the line to enable cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()
