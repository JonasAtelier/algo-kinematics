# algo-kinematics

Calculate how a robot moves from its joint angles or wheel speeds. These C99 libraries cover arms and wheeled bases.

**Contents** · [Choose a library](#choose-a-library) · [Try the example](#try-the-example) · [Add it to your project](#add-it-to-your-project)

## Choose a library

| Library | What it does | Example and API |
| --- | --- | --- |
| [arm-kinematics](arm-kinematics/) | Convert between joint angles and the position and orientation of an arm’s tool. | [arm6](arm-kinematics/examples/arm6.c) · [API](arm-kinematics/include/kin.h) |
| [arm-dynamics](arm-dynamics/) | Calculate joint torques, including those needed to hold an arm against gravity. | [gravity](arm-dynamics/examples/gravity.c) · [API](arm-dynamics/include/dyn.h) |
| [wheel-odometry](wheel-odometry/) | Convert wheel speeds into movement estimates for differential, mecanum and Ackermann bases. | [rover](wheel-odometry/examples/rover.c) · [API](wheel-odometry/include/mob.h) |

## Try the example

From this repository’s root, use Make and a C99 compiler. No robot hardware is needed.

```sh
make -C arm-kinematics run
```

The six-axis arm example calculates the pose of the tool from joint angles, then solves for angles that reach the same pose. [Example source](arm-kinematics/examples/arm6.c)

## Add it to your project

For plain C, compile the source files in the library’s `src/` directory and add its `include/` directory to your compiler’s include path. For a CMake application named `app`, place this checkout inside your project and use the following lines.

```cmake
add_subdirectory(algo-kinematics/arm-kinematics)
target_link_libraries(app PRIVATE algo::arm-kinematics)
```

For ESP-IDF, add the library directory to `EXTRA_COMPONENT_DIRS` and its name to the consuming component’s `REQUIRES`. Select either the individual libraries or the entire repository to avoid duplicate symbols.

<details>
<summary>Fetch just this library from GitHub with CMake</summary>

```cmake
include(FetchContent)
FetchContent_Declare(algo_arm_kinematics
    GIT_REPOSITORY https://github.com/JonasAtelier/algo-kinematics.git
    GIT_TAG        stable
    SOURCE_SUBDIR  arm-kinematics)
FetchContent_MakeAvailable(algo_arm_kinematics)
target_link_libraries(app PRIVATE algo::arm-kinematics)
```

</details>

Run `make check` from the repository root to test the libraries. [MIT license](LICENSE)

[Browse the workshop catalog · Portable C libraries](https://github.com/JonasAtelier/jonas-atelier-libraries#portable-c-libraries)
