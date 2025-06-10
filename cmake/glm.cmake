# use either system or FetchContent package
find_package(glm 1.0.1 QUIET)
if (NOT glm_FOUND)
    set(GLM_BUILD_TEST OFF)
    set(GLM_BUILD_LIBRARY OFF)
    set(GLM_BUILD_INSTALL OFF)
    set(GLM_ENABLE_LANG_EXTENSIONS ${CMAKE_CXX_EXTENSIONS})
    set(GLM_DISABLE_AUTO_DETECTION OFF)
    set(GLM_FORCE_PURE OFF)

    include(FetchContent)
    FetchContent_Declare(glm
        GIT_REPOSITORY "https://github.com/g-truc/glm.git"
        GIT_TAG "1.0.1"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(glm)
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE
    "GLM_FORCE_CXX20"
    "GLM_FORCE_INLINE"
    "GLM_FORCE_INTRINSICS")
target_link_libraries(${PROJECT_NAME} PRIVATE glm::glm-header-only)