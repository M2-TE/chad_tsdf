cmake_minimum_required(VERSION 3.21)
project(chad_tsdf LANGUAGES CXX)

# settings
option(CHAD_PREFER_MOLD "Prefer to use mold linker (Linux only)" ON)
option(CHAD_PREFER_CCACHE "Prefer to use ccache (Linux only)" ON)
option(CHAD_USE_FAST_MATH "Enable aggressive math optimizations" ON)
option(CHAD_USE_STRICT_COMPILATION "Force all warnings to emit errors" OFF)
include("cmake/options_global.cmake")
include("cmake/options_compiler.cmake")

# create chad tsdf library
file(GLOB_RECURSE CPP_SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/*.cpp")
add_library(${PROJECT_NAME} ${CPP_SOURCE_FILES})
add_library(chad::tsdf ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
include("cmake/fmt.cmake")
include("cmake/gtl.cmake")
include("cmake/glm.cmake")
include("cmake/eigen.cmake")
include("cmake/morton.cmake")

# compile executable if top level project
if (PROJECT_IS_TOP_LEVEL)
    add_executable(chad_executable "src/chad/main.cpp")
    target_link_libraries(chad_executable PRIVATE chad::tsdf)
    target_link_libraries(chad_executable PRIVATE glm::glm)
    target_compile_definitions(chad_executable PRIVATE
        "GLM_FORCE_ALIGNED_GENTYPES"
        "GLM_FORCE_INTRINSICS")
endif()