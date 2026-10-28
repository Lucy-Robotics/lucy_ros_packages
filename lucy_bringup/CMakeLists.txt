cmake_minimum_required(VERSION 3.8)
project(lucy_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install system scripts
install(PROGRAMS
  system_scripts/launch_lucy.sh
  system_scripts/stop_lucy.sh
  system_scripts/check_lucy.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  system_scripts/lucy_workspace.zsh.inc
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE "")
  # uncrustify/cppcheck are often missing or flaky on RoboStack macOS/Windows;
  # Linux CI (lucy_ws scripts/ci_colcon_test.sh) runs the remaining linters.
  if(APPLE OR WIN32)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_uncrustify
      ament_cmake_cppcheck
    )
  endif()
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_launch_syntax test/test_launch_syntax.py
    TIMEOUT 120
  )
  ament_add_pytest_test(test_mcu_bridge_bringup test/test_mcu_bridge_bringup.py
    TIMEOUT 60
  )
endif()

ament_package()

