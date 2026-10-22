# Makes this library usable directly, instead of needing a wrapper in the
# consuming project: as an ESP-IDF component, or in plain CMake through
# add_subdirectory() or FetchContent as the target algo::arm-kinematics.
# Ignored by the Makefile build, which stays the way to build and test this
# library on a host.
if(COMMAND idf_component_register)
    idf_component_register(SRCS "src/kin.c"
                           INCLUDE_DIRS "include")
    return()
endif()

add_library(algo-arm-kinematics src/kin.c)
add_library(algo::arm-kinematics ALIAS algo-arm-kinematics)
target_include_directories(algo-arm-kinematics PUBLIC include)
set_target_properties(algo-arm-kinematics PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_link_libraries(algo-arm-kinematics PRIVATE m)
endif()
