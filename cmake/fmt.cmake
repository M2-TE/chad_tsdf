# use either system or FetchContent package
find_package(fmt 11.2.0 QUIET)
if (NOT fmt_FOUND)
    set(FMT_PEDANTIC OFF)
    set(FMT_WERROR OFF)
    set(FMT_DOC OFF)
    set(FMT_INSTALL OFF)
    set(FMT_TEST OFF)
    set(FMT_FUZZ OFF)
    set(FMT_CUDA_TEST OFF)
    set(FMT_OS ON)
    set(FMT_MODULE OFF)
    set(FMT_SYSTEM_HEADERS ON)
    set(FMT_UNICODE ON)

    include(FetchContent)
    FetchContent_Declare(fmt
        GIT_REPOSITORY "https://github.com/fmtlib/fmt.git"
        GIT_TAG "11.2.0"
        GIT_SHALLOW ON
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL
        SYSTEM)
    FetchContent_MakeAvailable(fmt)
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC fmt::fmt)