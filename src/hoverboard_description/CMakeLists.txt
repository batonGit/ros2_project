cmake_minimum_required(VERSION 3.8)
project(hoverboard_description)

# Find ROS 2 packages
# ament_cmake provides the necessary infrastructure for ROS 2 build system
find_package(ament_cmake REQUIRED)

# Install directories containing description files and launch files
# These directories will be copied to the 'share' directory of the installed package.
# The user remembered this part correctly.
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# Add checks for linting if testing is enabled
# This is good practice to include, similar to other packages
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Depending on your linters configuration, you might also need ament_lint_common,
  # but ament_lint_auto often takes care of finding its dependencies.
  # find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export information about this package for other packages to depend on it
ament_package()