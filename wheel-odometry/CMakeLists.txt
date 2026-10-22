# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::wheel-odometry.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "src/mob.c"
                           INCLUDE_DIRS "include")
    return()
endif()

add_library(algo-wheel-odometry src/mob.c)
add_library(algo::wheel-odometry ALIAS algo-wheel-odometry)
target_include_directories(algo-wheel-odometry PUBLIC include)
set_target_properties(algo-wheel-odometry PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_link_libraries(algo-wheel-odometry PRIVATE m)
endif()
