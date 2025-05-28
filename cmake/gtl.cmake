# use either system or FetchContent package
find_package(gtl QUIET)
if (NOT gtl_FOUND)
    include(FetchContent)
    FetchContent_Declare(gtl
        GIT_REPOSITORY "https://github.com/greg7mdp/gtl.git"
        GIT_TAG "v1.2.0"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(gtl)
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC gtl)