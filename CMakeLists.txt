cmake_minimum_required(VERSION 3.15)
project(chad LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include (_global)

file(GLOB_RECURSE CPP_SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_executable(${PROJECT_NAME} ${CPP_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
# todo: some options
option(CHAD_ENABLE_ENTRY "Enables entrypoint in main.cpp" ON)

# fetch project dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET OFF) # enable git output for FetchContent steps
set(FETCHCONTENT_UPDATES_DISCONNECTED ON) # speed up consecutive config runs
include(glm)
include(fmt)
include(phmap)
set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "Faster config after FetchContent has run once" FORCE)
