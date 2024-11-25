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

if (NOT DEFINED CHAD_NORM_NEIGH_UPPER)
    set(CHAD_NORM_NEIGH_UPPER 20)
endif()
if (NOT DEFINED CHAD_NORM_NEIGH_LOWER)
    set(CHAD_NORM_NEIGH_LOWER 2)
endif()
if (NOT DEFINED CHAD_NORM_MIN_NEIGH)
    set(CHAD_NORM_MIN_NEIGH 3)
endif()
if (NOT DEFINED CHAD_NORM_RADIUS_MOD)
    set(CHAD_NORM_RADIUS_MOD 1.5)
endif()
if (NOT DEFINED CHAD_LEAF_RESOLUTION)
    set(CHAD_LEAF_RESOLUTION 0.1)
endif()
if (NOT DEFINED CHAD_LEAF_BITS)
    set(CHAD_LEAF_BITS 8)
endif()

# create executable/library
file(GLOB_RECURSE CPP_SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
if (CHAD_TOP_LEVEL)
    add_executable(${PROJECT_NAME} ${CPP_SOURCE_FILES})
    target_compile_definitions(${PROJECT_NAME} PRIVATE CHAD_MAIN)
    set_target_properties(${PROJECT_NAME} PROPERTIES 
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/bin/"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/bin/"
        CXXFLAGS "${CMAKE_CXX_FLAGS} ${WARNING_FLAGS}")
else()
    add_library(${PROJECT_NAME} ${CPP_SOURCE_FILES})
    add_library(chad::tsdf ALIAS ${PROJECT_NAME})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_compile_definitions(${PROJECT_NAME} PUBLIC CHAD_NORM_NEIGH_UPPER=${CHAD_NORM_NEIGH_UPPER})
target_compile_definitions(${PROJECT_NAME} PUBLIC CHAD_NORM_NEIGH_LOWER=${CHAD_NORM_NEIGH_LOWER})
target_compile_definitions(${PROJECT_NAME} PUBLIC CHAD_NORM_MIN_NEIGH=${CHAD_NORM_MIN_NEIGH})
target_compile_definitions(${PROJECT_NAME} PUBLIC CHAD_NORM_MAX_NEIGH=${CHAD_NORM_MAX_NEIGH})
target_compile_definitions(${PROJECT_NAME} PUBLIC CHAD_NORM_RADIUS_MOD=${CHAD_NORM_RADIUS_MOD})
target_compile_definitions(${PROJECT_NAME} PUBLIC LEAF_RESOLUTION=${CHAD_LEAF_RESOLUTION})
target_compile_definitions(${PROJECT_NAME} PUBLIC LEAF_BITS=${CHAD_LEAF_BITS})
target_compile_definitions(${PROJECT_NAME} PUBLIC CHAD_POPCOUNT_INSTRUCTION=${CHAD_POPCOUNT_INSTRUCTION})

# fetch project dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET OFF) # enable git output for FetchContent steps
set(FETCHCONTENT_UPDATES_DISCONNECTED ON) # speed up consecutive config runs
include(glm)
include(fmt)
include(phmap)
include(morton)
include(vulkan)
include(vma)
include(shaders)