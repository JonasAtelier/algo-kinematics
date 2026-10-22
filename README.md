# algo-kinematics

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Kinematics for robots — three small libraries in portable C99, each one
header and one `.c` file, no heap. Arms and wheeled bases: where it is, and
what it takes to move it.

## Libraries

| Library | What it does | Pick it when |
|---|---|---|
| [arm-kinematics](arm-kinematics) | Forward and inverse kinematics for serial arms | you need joint angles for a hand pose, or the reverse |
| [arm-dynamics](arm-dynamics) | Rigid-body dynamics for serial arms | you need the torque to hold or move the arm — gravity compensation |
| [wheel-odometry](wheel-odometry) | Wheel kinematics and dead reckoning for mobile bases | a differential, mecanum or Ackermann base needs wheel speeds or a pose |

## Use one library

Every library stands on its own: no shared headers, nothing from one library
needed by another. Take the one you need and leave the rest. Three ways in.

**Copy the files.** `arm-kinematics/include/kin.h` and
`arm-kinematics/src/kin.c`
into your project, built with your own sources. That is the whole install.

**CMake.** Every library is its own target, `algo::<folder>`. Add just the
folder you need:

```cmake
add_subdirectory(algo-kinematics/arm-kinematics)
target_link_libraries(app PRIVATE algo::arm-kinematics)
```

or, without a copy in your tree:

```cmake
include(FetchContent)
FetchContent_Declare(algo_arm_kinematics
    GIT_REPOSITORY https://github.com/JonasAtelier/algo-kinematics.git
    GIT_TAG        stable
    SOURCE_SUBDIR  arm-kinematics)
FetchContent_MakeAvailable(algo_arm_kinematics)
target_link_libraries(app PRIVATE algo::arm-kinematics)
```

`add_subdirectory(algo-kinematics)`, or FetchContent without `SOURCE_SUBDIR`,
adds every library's target instead; link the ones you use. Examples and tests
only build when this repo is the top-level project.

**ESP-IDF.** Point `EXTRA_COMPONENT_DIRS` at one library's folder. The
component is named after the folder:

```cmake
# top-level CMakeLists.txt of your IDF project, before project()
list(APPEND EXTRA_COMPONENT_DIRS path/to/algo-kinematics/arm-kinematics)
```

then `REQUIRES arm-kinematics` in your component's `idf_component_register()`.

Pointing it at the repo root instead gives you **one** component holding
every library, named after the folder (`algo-kinematics`): IDF does not look
inside a folder that has its own `CMakeLists.txt`. Use one or the other,
never both — every symbol would be defined twice.

## Build and test

```sh
make check    # every library's own tests, stopping at the first failure
make clean
```

Or with CMake and CTest. CTest runs the C tests only; the C++ linkage tests
run under `make check`.

```sh
cmake -B build && cmake --build build -j2 && ctest --test-dir build
```

## License

MIT. See [LICENSE](LICENSE).
