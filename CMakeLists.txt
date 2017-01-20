cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

add_subdirectory(gazebo_plugins)
add_subdirectory(reach-client)
add_subdirectory(stand-client)

include(AddUninstallTarget)
