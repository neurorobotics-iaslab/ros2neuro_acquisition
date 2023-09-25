cmake_minimum_required(VERSION 3.8)
project(ros2neuro_acquisition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ---- dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2neuro_msgs REQUIRED)
find_package(ros2neuro_data REQUIRED)
find_package(rclcpp REQUIRED)

# ---- add library for the device
add_library(Device SHARED
  src/Device.cpp)
ament_target_dependencies(
  Device
  "rclcpp"
  "ros2neuro_data"
)
target_include_directories(Device PUBLIC include)
ament_export_libraries(Device)
install(
  TARGETS Device
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---- add library for the Acquisition
add_library(${PROJECT_NAME} SHARED
  src/Acquisition.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "ros2neuro_data"
  "ros2neuro_msgs"
  "pluginlib")
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME}
  Device)
ament_export_libraries(${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---- add test plugin cpp file
add_executable(test_plugin src/test_plugin.cpp)
target_include_directories(test_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(test_plugin
  Device)
target_compile_features(test_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_plugin
  "pluginlib"
  "ros2neuro_msgs"
  "ros2neuro_data"
  "rclcpp"
)
install(TARGETS test_plugin
  DESTINATION lib/${PROJECT_NAME})

# ---- cpp file for acquisition node
add_executable(acquisition src/acquisition.cpp)
target_include_directories(acquisition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(acquisition
  ${PROJECT_NAME})
target_compile_features(acquisition PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  acquisition
  "pluginlib"
  "ros2neuro_msgs"
  "ros2neuro_data"
  "rclcpp"
)
install(TARGETS acquisition
  DESTINATION lib/${PROJECT_NAME})

# --- final installations/exporting
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

# ---- to remove unused-parameters warnings
if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-parameter)
endif()


ament_package()
