set(GTSAM_ENABLE_BOOST_SERIALIZATION ON)
set(GTSAM_USE_BOOST_FEATURES ON)

include(FetchContent)
FetchContent_Declare(gtsam
    GIT_REPOSITORY "https://github.com/borglab/gtsam.git"
    GIT_TAG "master"
    GIT_SHALLOW ON
    OVERRIDE_FIND_PACKAGE
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(gtsam)
target_link_libraries(${PROJECT_NAME} PRIVATE gtsam)