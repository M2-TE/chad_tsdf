# SPIR-V for shader reflection
set(SPIRV_REFLECT_EXECUTABLE     OFF)
set(SPIRV_REFLECT_STATIC_LIB     ON)
set(SPIRV_REFLECT_BUILD_TESTS    OFF)
set(SPIRV_REFLECT_ENABLE_ASSERTS OFF)
set(SPIRV_REFLECT_ENABLE_ASAN    OFF)
FetchContent_Declare(spirv-reflect
    GIT_REPOSITORY "https://github.com/KhronosGroup/SPIRV-Reflect.git"
    GIT_TAG "vulkan-sdk-1.3.290.0"
    GIT_SHALLOW ON
    GIT_SUBMODULES "")
FetchContent_MakeAvailable(spirv-reflect)
target_link_libraries(${PROJECT_NAME} PRIVATE spirv-reflect-static)

# SPVRC for shader compilation and embedding
set(SPVRC_SHADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set(SPVRC_SHADER_ENV "vulkan1.1")
FetchContent_Declare(spvrc
    GIT_REPOSITORY "https://github.com/M2-TE/spvrc.git"
    GIT_TAG "main"
    GIT_SHALLOW ON)
FetchContent_MakeAvailable(spvrc)
target_link_libraries(${PROJECT_NAME} PRIVATE spvrc::spvrc)