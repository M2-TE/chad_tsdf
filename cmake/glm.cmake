set(GLM_DISABLE_AUTO_DETECTION  OFF)
set(GLM_ENABLE_LANG_EXTENSIONS  ${CMAKE_CXX_EXTENSIONS})
set(GLM_ENABLE_FAST_MATH        ${CROSS_PLATFORM_NONDETERMINISTIC})
set(GLM_ENABLE_CXX_20           ON)
set(GLM_FORCE_PURE              OFF)
cmake_host_system_information(RESULT GLM_ENABLE_SIMD_SSE2 QUERY HAS_SSE2)
# option(GLM_ENABLE_SIMD_SSE3 "Enable SSE3 optimizations" OFF)
# option(GLM_ENABLE_SIMD_SSSE3 "Enable SSSE3 optimizations" OFF)
# option(GLM_ENABLE_SIMD_SSE4_1 "Enable SSE 4.1 optimizations" OFF)
# option(GLM_ENABLE_SIMD_SSE4_2 "Enable SSE 4.2 optimizations" OFF)
# option(GLM_ENABLE_SIMD_AVX "Enable AVX optimizations" OFF)
# option(GLM_ENABLE_SIMD_AVX2 "Enable AVX2 optimizations" OFF)
FetchContent_Declare(glm GIT_REPOSITORY "https://github.com/g-truc/glm.git" GIT_TAG "1.0.1" GIT_SHALLOW ON)
FetchContent_MakeAvailable(glm)
target_compile_definitions(${PROJECT_NAME} PRIVATE 
    "GLM_FORCE_DEPTH_ZERO_TO_ONE"
    "GLM_FORCE_ALIGNED_GENTYPES"
    "GLM_FORCE_INTRINSICS")
target_link_libraries(${PROJECT_NAME} PUBLIC glm::glm)