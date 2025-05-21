# use either system or FetchContent package
find_package(libmorton QUIET)
if (NOT libmorton_FOUND)
    set(BUILD_TESTING OFF)

    include(FetchContent)
    FetchContent_Declare(libmorton
        GIT_REPOSITORY "https://github.com/Forceflow/libmorton.git"
        GIT_TAG "v0.2.12"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL
        SYSTEM)
    FetchContent_MakeAvailable(libmorton)
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC libmorton::libmorton)