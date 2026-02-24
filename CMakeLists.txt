cmake_minimum_required(VERSION 3.24)

set(PROJECT_NAME mc_joystick_plugin)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL "")
set(PROJECT_VERSION 0.0.1)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CMAKE_CXX_STANDARD 17)

include(cmake/base.cmake)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})
option(AUTOLOAD_${PROJECT_NAME}_PLUGIN "Automatically load ${PROJECT_NAME} plugin" OFF)

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()


# Configure and install the plugin configuration
configure_file(etc/${PROJECT_NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(joystick_inputs_HDR_DIR
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/mc_joystick_plugin>$<INSTALL_INTERFACE:include/mc_joystick_plugin>)

set(joystick_inputs_HDR
  ${joystick_inputs_HDR_DIR}/joystick_inputs.h
)

# header-only
add_library(joystick_inputs INTERFACE)
if(COMMAND target_sources)
  target_sources(joystick_inputs INTERFACE ${joystick_inputs_HDR})
endif()
add_library(joystick_inputs_properties ALIAS joystick_inputs)

install(TARGETS joystick_inputs
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

set(plugin_SRC
  src/plugin.cpp
  src/joystick/joystick.cc
  )

set(plugin_HDR
  src/plugin.h
  src/joystick/joystick.hh
)

# The add_plugin macro is provided by mc_rtc
add_plugin(${PROJECT_NAME} "${plugin_SRC}" "${plugin_HDR}")

target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC joystick_inputs)

install(
  DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

