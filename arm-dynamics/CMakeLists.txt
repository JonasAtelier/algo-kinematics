# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::arm-dynamics.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "src/dyn.c"
                           INCLUDE_DIRS "include")
    return()
endif()

add_library(algo-arm-dynamics src/dyn.c)
add_library(algo::arm-dynamics ALIAS algo-arm-dynamics)
target_include_directories(algo-arm-dynamics PUBLIC include)
set_target_properties(algo-arm-dynamics PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_link_libraries(algo-arm-dynamics PRIVATE m)
endif()
