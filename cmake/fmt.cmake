set(FMT_PEDANTIC OFF)
set(FMT_WERROR OFF)
set(FMT_DOC OFF)
set(FMT_INSTALL OFF)
set(FMT_TEST OFF)
set(FMT_FUZZ OFF)
set(FMT_CUDA_TEST OFF)
set(FMT_OS ON)
set(FMT_MODULE OFF)
set(FMT_SYSTEM_HEADERS OFF)
set(FMT_UNICODE ON)

# check if fmt is already present
FetchContent_Declare(fmt
    GIT_REPOSITORY "https://github.com/fmtlib/fmt.git"
    GIT_TAG "11.0.2"
    GIT_SHALLOW ON
    OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(fmt)
target_link_libraries(${PROJECT_NAME} PUBLIC fmt::fmt)