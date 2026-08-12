cmake_minimum_required(VERSION 3.21)
project(chad_tsdf LANGUAGES CXX)

# settings
include("cmake/settings.cmake")
option(CHAD_PREFER_MOLD "Prefer to use mold linker" ON)
option(CHAD_USE_FAST_MATH "Enable aggressive math optimizations" ON)
option(CHAD_USE_ARCH_NATIVE "Compile for the native CPU architecture" ON)
option(CHAD_USE_STRICT_COMPILATION "Force all warnings to emit errors" OFF)
# option(CHAD_USE_HUGE_PAGES "Allocate huge pages (~2MiB) instead of the standard size (~4KiB)" OFF)

# create chad tsdf library
add_library(${PROJECT_NAME}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/tsdf_map.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/optimizer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/virtual_array.cpp")
add_library(chad::tsdf ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
target_precompile_headers(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/chad/pch.cpp")
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# compiler/linker options across different vendors
target_compile_options(${PROJECT_NAME} PRIVATE
    # warnings
    $<$<CXX_COMPILER_ID:MSVC>: "/W4">
    $<$<CXX_COMPILER_ID:GNU,Clang>: "-Wall" "-Wextra" "-Wpedantic" "-Wno-unused-parameter">
    # warning errors
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<BOOL:${CHAD_USE_STRICT_COMPILATION}>>: "/WX">
    $<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<BOOL:${CHAD_USE_STRICT_COMPILATION}>>: "-Werror">
    # fast math optimizations
    $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<BOOL:${CHAD_USE_FAST_MATH}>>: "/fp:fast">
    $<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<BOOL:${CHAD_USE_FAST_MATH}>>: "-ffast-math">
    # SIMD instruction set (native or BMI2 and SSE2)
    $<$<CXX_COMPILER_ID:MSVC>: "/arch:AVX2">
    $<$<CXX_COMPILER_ID:GNU,Clang>:$<IF:$<BOOL:${CHAD_USE_ARCH_NATIVE}>, "-march=native", "-mbmi2" "-msse2">>
    # misc
    $<$<CXX_COMPILER_ID:GNU,Clang>: "-mfpmath=sse">
    # $<$<CXX_COMPILER_ID:Clang>: "-fexperimental-library"> # to access parallel algorithms in <execution> (disabled due to perf regression)
    $<$<CXX_COMPILER_ID:MSVC>: "/diagnostics:classic" "/Zc:__cplusplus" "/Zc:inline" "/fp:except-" "/FC" "/Gm-" "/MP" "/nologo">)
target_link_options(${PROJECT_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>: "/SUBSYSTEM:CONSOLE">
    $<$<CXX_COMPILER_ID:GNU,Clang>: "-pthread">)
# handle dependencies
include("cmake/fmt.cmake")
include("cmake/gtl.cmake")
include("cmake/glm.cmake")
include("cmake/gtsam.cmake")

# compile executable if top level project
if (PROJECT_IS_TOP_LEVEL)
    add_executable(chad_executable "src/chad/main.cpp")
    target_link_libraries(chad_executable PRIVATE chad::tsdf)
    target_link_libraries(chad_executable PRIVATE fmt::fmt)
    target_link_libraries(chad_executable PRIVATE glm::glm-header-only)
    target_compile_features(chad_executable PRIVATE cxx_std_20)
endif()
