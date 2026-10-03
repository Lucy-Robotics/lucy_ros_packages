cmake_minimum_required(VERSION 3.8)
project(lucy_ros2_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  lucy_ros2_control
  SHARED
  src/lucy_system.cpp
  src/joint_config.cpp
)

target_compile_features(lucy_ros2_control PUBLIC cxx_std_17)
target_include_directories(lucy_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
$<INSTALL_INTERFACE:include/lucy_ros2_control>
)
# Link the namespaced CMake targets rather than ament_target_dependencies.
# ament_target_dependencies expands to each dependency's full ament link list,
# which includes its <msg>__rosidl_generator_py libraries. Those are built to be
# loaded inside a Python interpreter: they carry undefined _Py* symbols and
# deliberately do not link libpython. Dragging them into a C++ plugin makes
# macOS's flat-namespace dlopen fail to resolve them, so pluginlib rejects the
# hardware component with "symbol not found in flat namespace
# '_PyExc_RuntimeError'" and controller_manager never loads any hardware.
target_link_libraries(lucy_ros2_control PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

if(APPLE)
  # Those dependencies expand to each message package's __rosidl_generator_py
  # library. Those are built to be loaded inside a Python interpreter: they carry
  # undefined _Py* symbols and deliberately do not link libpython, because in
  # `python3` the symbols are already in the process.
  #
  # This plugin uses no symbol from any of them, but they still land in its load
  # commands. ros2_control_node is pure C++, so when pluginlib dlopen()s the
  # plugin the flat-namespace lookup for '_PyExc_RuntimeError' finds nothing and
  # the hardware component is rejected — controller_manager then loads no
  # hardware, spawns no controllers, and nothing publishes /joint_states.
  #
  # -dead_strip_dylibs drops dylibs no symbol is used from, removing them before
  # they can be loaded. Linux binds these lazily and never hits this, so keep the
  # flag APPLE-only.
  target_link_options(lucy_ros2_control PRIVATE "LINKER:-dead_strip_dylibs")
endif()

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lucy_ros2_control.xml)


# Installing
install(
  DIRECTORY src/include/
  DESTINATION include/lucy_ros2_control
)

# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/lucy_ros2_control
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/lucy_ros2_control
# )
install(TARGETS lucy_ros2_control
  EXPORT export_lucy_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # cpplint excluded: noise on the inherited ros2_control style; copyright lint
  # expects the standard GPLv3 header block (matches lucy_bringup / package.xml).
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
  )
  # uncrustify/cppcheck are often missing or flaky on RoboStack macOS/Windows;
  # Linux CI (lucy_ws scripts/ci_colcon_test.sh) runs the remaining linters.
  if(APPLE OR WIN32)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_uncrustify
      ament_cmake_cppcheck
    )
  endif()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # RoboStack macOS: gtest_vendor sources pick up mismatched gtest headers from
  # $CONDA_PREFIX/include (TestInfo API drift). GTests are enabled on Linux only;
  # lucy_ws CI (scripts/ci_colcon_test.sh on linux-64) runs them. Re-enable on
  # APPLE/WIN32 when RoboStack ships consistent gtest headers across platforms.
  if(NOT APPLE AND NOT WIN32)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_position_limit_clamp test/test_position_limit_clamp.cpp)
    target_include_directories(test_position_limit_clamp PRIVATE
      ${PROJECT_SOURCE_DIR}/src/include
    )

    # Pure on_init helpers: compile the module straight into the test (no ROS node
    # needed); depends on hardware_interface for ComponentInfo / lexical_casts.
    ament_add_gtest(test_joint_config
      test/test_joint_config.cpp
      src/joint_config.cpp
    )
    target_include_directories(test_joint_config PRIVATE
      ${PROJECT_SOURCE_DIR}/src/include
    )
    ament_target_dependencies(test_joint_config hardware_interface)
  endif()
endif()

ament_package()
