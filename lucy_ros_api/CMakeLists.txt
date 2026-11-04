cmake_minimum_required(VERSION 3.8)
project(lucy_ros_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (generic interface: base, ROS adapter, mock adapter)
ament_python_install_package(${PROJECT_NAME})

# Install documentation
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# Install README in share
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_generic_interface test/test_generic_interface.py
    TIMEOUT 30
    PYTHON_EXECUTABLE "${PYTHON3_EXECUTABLE}"
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
