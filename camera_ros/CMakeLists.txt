cmake_minimum_required(VERSION 3.8)
project(camera_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# Bind-mounted / persisted build dirs can leave .../ament_cmake_python/<pkg>/<pkg> as a directory
# from an interrupted build; the next build then fails creating the symlink ("Is a directory").
if(TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  add_custom_target(_camera_ros_clear_stale_ament_python_symlink
    COMMAND bash -c "rm -rf -- '${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/${PROJECT_NAME}'"
    VERBATIM
    COMMENT "Remove stale path before ament_cmake_python symlink (Docker bind mounts)"
  )
  add_dependencies(ament_cmake_python_symlink_${PROJECT_NAME} _camera_ros_clear_stale_ament_python_symlink)
endif()

# Install Python scripts
install(PROGRAMS
  scripts/camera_publisher.py
  scripts/camera_stream_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(FILES
  launch/camera.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install README
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Install test files
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  # Install pytest.ini for coverage configuration
  install(FILES
    pytest.ini
    DESTINATION share/${PROJECT_NAME}
  )

  # Add pytest tests with coverage
  ament_add_pytest_test(test_camera_publisher test/test_camera_publisher.py
    TIMEOUT 60
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts:${CMAKE_CURRENT_SOURCE_DIR}"
  )

  ament_add_pytest_test(test_camera_stream_controller test/test_camera_stream_controller.py
    TIMEOUT 60
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/scripts:${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
