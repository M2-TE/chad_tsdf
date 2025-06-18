# policies
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW) # option() honors normal variables
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW) # IPO/LTO

# FetchContent settings
set(FETCHCONTENT_QUIET OFF) # enable git output for FetchContent steps
set(FETCHCONTENT_UPDATES_DISCONNECTED ON) # speed up consecutive config runs

# IPO/LTO support
include(CheckIPOSupported)
check_ipo_supported(RESULT CMAKE_INTERPROCEDURAL_OPTIMIZATION LANGUAGES CXX)
message(STATUS "IPO/LTO enabled: ${CMAKE_INTERPROCEDURAL_OPTIMIZATION}")

# platform-specific
if (MSVC)
    set(CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE "x64")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>") # static msvc runtime lib
elseif (UNIX)
    # enable mold linker if present
    find_program(MOLD_FOUND mold)
    if (CHAD_PREFER_MOLD AND MOLD_FOUND)
        message(STATUS "Using mold linker")
        set(CMAKE_LINKER_TYPE MOLD)
    endif()

    # enable ccache if present
    find_program(CCACHE_FOUND ccache)
    if (CHAD_PREFER_CCACHE AND CCACHE_FOUND)
        message(STATUS "Using ccache")
        set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
    endif()
endif()
