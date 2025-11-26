set(BRISK_BUILD_DEMO OFF)
set(BUILD_SHARED_LIBS OFF)
include(FetchContent)
FetchContent_Declare(brisk
    GIT_REPOSITORY "https://github.com/ethz-mrl/brisk.git"
    GIT_TAG "1ef8b42a5c2fdd0e0c976f3ab7b179806381f570"
    GIT_SHALLOW ON
    OVERRIDE_FIND_PACKAGE
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(brisk)
target_link_libraries(${PROJECT_NAME} PUBLIC brisk::brisk)