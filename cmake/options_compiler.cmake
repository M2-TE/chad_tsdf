# compiler options
if (MSVC)
    add_compile_options(
        "/diagnostics:classic"
        "/Zc:__cplusplus"
        "/Zc:inline"
        "/fp:except-"
        "/FC"
        "/Gm-"
        "/MP"
        "/W4"
        "/nologo")
    if (CHAD_USE_FAST_MATH)
        add_compile_options("/fp:fast")
    else()
        add_compile_options("/fp:precise")
    endif()
    if (CHAD_USE_STRICT_COMPILATION)
        add_compile_options("/WX")
    endif()
elseif (UNIX)
    # global compile options
    add_compile_options(
        "-Wall"
        "-Wextra"
        "-Wpedantic"
        "-mbmi2")
    if (CHAD_USE_FAST_MATH)
        # add_compile_options("-ffast-math") # currently broken due to gtsam
    endif()
    if (CHAD_USE_STRICT_COMPILATION)
        add_compile_options("-Werror")
    endif()

    # compiler specific flags
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    endif()
endif()

# linker options
if (MSVC)
    add_link_options("/SUBSYSTEM:CONSOLE")
elseif (UNIX)
    add_link_options("-pthread")
endif()