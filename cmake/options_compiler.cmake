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
        "-mbmi2"
        )
    if (CHAD_USE_FAST_MATH)
        add_compile_options("-ffast-math")
    endif()
    if (CHAD_USE_STRICT_COMPILATION)
        add_compile_options("-Werror")
    endif()

    # compiler specific flags
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options("-Wno-stringop-overflow")
        add_compile_options("-ffp-contract=off")
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        # none so far
    endif()
endif()

# linker options
if (MSVC)
    add_link_options("/SUBSYSTEM:CONSOLE")
elseif (UNIX)
    add_link_options(
        "-pthread"
        "-fopenmp") # required by lvr2
endif()