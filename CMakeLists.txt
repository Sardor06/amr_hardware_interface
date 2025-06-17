cmake_minimum_required(VERSION 3.8)
project(marvin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)

set(DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# BUILD
add_library(
  marvin
  SHARED
  src/hardware_interface.cpp
)
target_compile_features(marvin PUBLIC cxx_std_17)
target_include_directories(marvin PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/marvin>
)
ament_target_dependencies(
  marvin PUBLIC
  ${DEPENDENCIES}
)
target_link_libraries(marvin PUBLIC serial) # todo

# VISIBILITY
target_compile_definitions(marvin PRIVATE "MARVIN_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface marvin.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS marvin
  EXPORT export_marvin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_marvin HAS_LIBRARY_TARGET)
ament_export_dependencies(${Dependency})
ament_package()
