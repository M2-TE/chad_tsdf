# use either system or FetchContent package
find_package(Eigen3 QUIET)
if (NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(eigen
        GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
        GIT_TAG "3.4.0"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL
        SYSTEM)
    FetchContent_MakeAvailable(eigen)
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)