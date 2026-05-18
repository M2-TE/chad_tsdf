# use either system or FetchContent package
find_package(gtsam QUIET)
if (NOT gtsam_FOUND)
    include(FetchContent)
    set(GTSAM_BUILD_DOCS OFF)
    set(GTSAM_BUILD_TESTS OFF)
    set(GTSAM_ENABLE_ASAN OFF)
    set(GTSAM_ENABLE_GEOGRAPHICLIB OFF)
    set(GTSAM_USE_BOOST_FEATURES OFF)
    set(GTSAM_ENABLE_BOOST_SERIALIZATION OFF)
    # general options
    set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF)
    set(GTSAM_BUILD_TYPE_POSTFIXES OFF)
    set(GTSAM_BUILD_WITH_MARCH_NATIVE ${CHAD_USE_ARCH_NATIVE})
    set(GTSAM_BUILD_WITH_PRECOMPILED_HEADERS ON)
    set(GTSAM_BUILD_UNSTABLE OFF)
    set(GTSAM_UNSTABLE_BUILD_PYTHON OFF)
    set(GTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX OFF)
    set(GTSAM_FORCE_SHARED_LIB OFF)
    set(GTSAM_FORCE_STATIC_LIB ON)
    # set(GTSAM_USE_QUATERNIONS ON)
    set(GTSAM_ALLOW_DEPRECATED_SINCE_V43 OFF)

    FetchContent_Declare(gtsam
        GIT_REPOSITORY "https://github.com/borglab/gtsam.git"
        GIT_TAG "4.3a1"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(gtsam)
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE gtsam)
