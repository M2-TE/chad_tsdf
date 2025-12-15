cmake_minimum_required(VERSION 3.21)
project(chad_tsdf LANGUAGES CXX)

# settings
option(CHAD_PREFER_MOLD "Prefer to use mold linker (Linux only)" ON)
option(CHAD_USE_FAST_MATH "Enable aggressive math optimizations" ON)
option(CHAD_USE_STRICT_COMPILATION "Force all warnings to emit errors" OFF)
option(CHAD_USE_HUGE_PAGES "Allocate huge pages (~2MiB) instead of the standard size (~4KiB)" OFF) # currently does nothing
include("cmake/options_global.cmake")
include("cmake/options_compiler.cmake")

# create chad tsdf library
set(CPP_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/tsdf_map.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/detail/lvr2.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/detail/virtual_array.cpp")
add_library(${PROJECT_NAME} ${CPP_SOURCE_FILES})
add_library(chad::tsdf ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_precompile_headers(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include/chad/detail/pch.hpp")
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# handle dependencies
include("cmake/fmt.cmake")
include("cmake/gtl.cmake")
include("cmake/glm.cmake")
# include("cmake/gtsam.cmake")
include("cmake/eigen.cmake") # only needed to test eigen api (lvr2 also needs it due to dep leak)
include("cmake/morton.cmake")
# include("cmake/opencv.cmake")
include("cmake/lvr2.cmake")

# compile executable if top level project
if (PROJECT_IS_TOP_LEVEL)
    add_executable(chad_executable "src/chad/main.cpp")
    target_link_libraries(chad_executable PRIVATE chad::tsdf)
    target_link_libraries(chad_executable PRIVATE glm::glm-header-only)
    target_compile_features(chad_executable PRIVATE cxx_std_23)
    target_precompile_headers(chad_executable REUSE_FROM ${PROJECT_NAME})
    # target_link_libraries(chad_executable PRIVATE Eigen3::Eigen) # Debug only
endif()
