cmake_minimum_required(VERSION 3.15)
project(chad_tsdf LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# check if project is top level
set(CHAD_TOP_LEVEL OFF)
if (PROJECT_IS_TOP_LEVEL)
    set(CHAD_TOP_LEVEL ON)
    include(_global)
endif()

# set build options
option(CHAD_ENABLE_ENTRY "Enables entrypoint in main.cpp" CHAD_TOP_LEVEL)
if (NOT DEFINED CHAD_POPCOUNT_INSTRUCTION)
    set(CHAD_POPCOUNT_INSTRUCTION std::popcount<uint8_t>)
endif()

# create executable/library
file(GLOB_RECURSE CPP_SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
if (CHAD_TOP_LEVEL)
    add_executable(${PROJECT_NAME} ${CPP_SOURCE_FILES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE CHAD_MAIN)
else()
    add_library(${PROJECT_NAME} ${CPP_SOURCE_FILES})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_definitions(${PROJECT_NAME} PRIVATE LEAF_RESOLUTION=0.1)
target_compile_definitions(${PROJECT_NAME} PRIVATE LEAF_BITS=8)
target_compile_definitions(${PROJECT_NAME} PRIVATE CHAD_POPCOUNT_INSTRUCTION=${CHAD_POPCOUNT_INSTRUCTION})

# fetch project dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET OFF) # enable git output for FetchContent steps
set(FETCHCONTENT_UPDATES_DISCONNECTED ON) # speed up consecutive config runs
include(glm)
include(fmt)
include(phmap)
include(morton)
if (CHAD_TOP_LEVEL)
    set(FETCHCONTENT_FULLY_DISCONNECTED ON CACHE BOOL "Faster config after FetchContent has run once" FORCE)
endif()
