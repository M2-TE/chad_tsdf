# use either system or FetchContent package
set(LVR2_USE_STATIC_LIBS ON)
find_package(LVR2 25.1.0 QUIET)
if (NOT LVR2_FOUND)
    set(WITH_CUDA OFF)

    include(FetchContent)
    FetchContent_Declare(lvr2
        GIT_REPOSITORY "https://github.com/uos/lvr2.git"
        GIT_TAG "25.1.0"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(lvr2)
endif()

add_definitions(${LVR2_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LVR2_LIBRARIES}
    "/usr/local/lib/liblvr2rply_static.a" # evil act #1
    "/usr/local/lib/liblvr2las_static.a"  # evil act #2
    )
target_include_directories(${PROJECT_NAME} PRIVATE ${LVR2_INCLUDE_DIRS})