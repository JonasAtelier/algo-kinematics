# dyn

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Rigid-body dynamics for robot arms, in one C file.

Two questions, and this library answers both:

- **Inverse** — I want the arm to move *like this*, so what torque does each
  motor need?
- **Forward** — the motors are pushing with *this* torque, so how does the arm
  move?

[`kin`](../arm-kinematics) answers where the arm is. This one answers what it
takes to get there — the weight it is holding up, the inertia it has to
accelerate, the Coriolis terms that shove a fast joint sideways.

- One header, one source file. Drop them into any C project.
- Only needs `libm`. No `malloc` — works fine on a microcontroller.
- Recursive Newton-Euler, the standard textbook recursion. No robot-specific
  maths to derive.

## Try it

```sh
make check    # run the tests
make run      # gravity-compensation torques for a small 2-link arm
```

To use it in your own project, copy `include/dyn.h` and `src/dyn.c`, link the
CMake target `algo::arm-dynamics`, or use this folder as an ESP-IDF component.
See [Use one library](../README.md#use-one-library).

## Describing your arm

The first four numbers are exactly kin's — the standard **Denavit–Hartenberg**
parameters. A `kin_link` and a `dyn_link` with the same four numbers are the
same arm, so if kin already works, copy them straight across:

| number  | in plain words |
|---------|----------------|
| `a`     | how long the link is |
| `d`     | how far the next joint sits along the current joint's axis |
| `alpha` | how much the next joint axis is twisted relative to this one |
| `theta` | a fixed angle offset baked into the joint |

`DYN_REVOLUTE` adds the joint value to `theta`, `DYN_PRISMATIC` adds it to `d`.

Then three more numbers, which are the ones kin never needed:

| field     | in plain words |
|-----------|----------------|
| `mass`    | what the link weighs, in kg. Include the motor bolted to it. |
| `com`     | where that weight sits, as an x-y-z offset in the link's own frame |
| `inertia` | how hard the link is to *spin*, a 3x3 tensor about its centre of mass |

**Where is the link's own frame?** Its origin sits at the far end of the link,
where the next joint is, with x pointing back along the link. So a mass bolted
at the tip is `com = {0, 0, 0}`, and the middle of a uniform link is
`com = {-a/2, 0, 0}` — half a link *back* along x.

The easy start is a **point mass at the tip**: `com` all zero, `inertia` all
zero. A point has no size, so it has nothing to spin. That is a perfectly good
first model and it gets gravity compensation right, which is what most people
came for:

```c
static const dyn_link arm2[] = {
    /*  d    a    alpha theta  type          mass  com          inertia */
    {  0.0, 0.30, 0.0,  0.0,   DYN_REVOLUTE, 1.5,  {0, 0, 0},   {0,0,0, 0,0,0, 0,0,0} },
    {  0.0, 0.25, 0.0,  0.0,   DYN_REVOLUTE, 0.8,  {0, 0, 0},   {0,0,0, 0,0,0, 0,0,0} },
};
static const dyn_chain arm = { arm2, 2, { 0.0, -9.81, 0.0 } };
```

When you want better, a uniform rod of length `a` is `com = {-a/2, 0, 0}` with
`m*a*a/12` on the two diagonal entries across the rod — that is what
`examples/gravity.c` uses. Anything more accurate comes out of your CAD package,
which will hand you a mass, a centre of mass and an inertia tensor per part.

`gravity` is the gravity vector in the base frame, so `{0, 0, -9.81}` for an arm
standing on the floor, or `{0, -9.81, 0}` for a flat one working in a vertical
plane. All zeros means no gravity at all, which is occasionally what you want.

## Gravity compensation in five lines

This is the use case. An arm with no gravity compensation sags below where you
commanded it, and the fix is to feed each motor the torque that holds the arm up
before you add whatever your controller wants on top:

```c
double hold[2];

dyn_gravity(&arm, q, hold);          /* q = where the arm is right now */

for (i = 0; i < 2; i++)
    command[i] = hold[i] + pid[i];   /* your controller only fights the error now */
```

Call it every cycle with the current joint angles. The torque changes as the arm
moves — an arm stretched out horizontally needs far more than one hanging
straight down.

## The functions

```c
dyn_gravity(&arm, q, tau);                /* what holds it still here */
dyn_inverse(&arm, q, qd, qdd, tau);       /* to move like this -> what torque */
dyn_forward(&arm, q, qd, tau, qdd);       /* with this torque -> how it moves */
dyn_mass_matrix(&arm, q, M);              /* how hard each joint is to accelerate */
```

| call | what it does |
|------|--------------|
| `dyn_gravity` | the torque that stops the arm falling at this pose. `dyn_inverse` with everything else zero |
| `dyn_inverse` | full inverse dynamics: gravity, inertia, Coriolis and centrifugal. `qd` and `qdd` may be `NULL`, meaning zero |
| `dyn_forward` | the acceleration a torque actually produces. Integrate it twice and you have a simulator |
| `dyn_mass_matrix` | `M(q)`, n by n and symmetric, row-major. For controllers that need it; ignore it otherwise |

All four return `DYN_OK` or `DYN_EARG`. `DYN_EARG` means a `NULL` pointer, an
`n` outside 1..`DYN_MAX_JOINTS`, a number that is not finite, or a negative
mass. `dyn_forward` also returns it if the mass matrix cannot be inverted, which
in practice means a link with no mass or no inertia in it.

## Frames and units

Everything is SI: metres, kilograms, radians, seconds.

| name | meaning | unit |
|------|---------|------|
| `q` | where each joint is | rad, or m for a prismatic joint |
| `qd` | how fast it is moving | rad/s, or m/s |
| `qdd` | how fast that is changing | rad/s², or m/s² |
| `tau` | what the motor pushes with | N m, or N for a prismatic joint |
| `mass` | what the link weighs | kg |
| `com` | where the weight sits, in the link frame | m |
| `inertia` | about the centre of mass, in the link frame | kg m² |
| `gravity` | in the base frame, pointing the way things fall | m/s² |

A prismatic joint's `tau` entry is a force in newtons, not a torque. Same array,
different unit — the same way `q` is metres for that joint.

## What it does not do

- No friction. Real joints have stiction and viscous drag, and they are often
  the same size as the terms this does model. Measure yours and add them.
- No motor or gearbox inertia. A geared motor's rotor inertia is multiplied by
  the square of the ratio and can dominate everything here. Add it to the
  diagonal of `M` yourself.
- No joint limits, no torque limits. Ask for an impossible acceleration and it
  will hand you the torque without comment.
- No contact and no external forces. The arm is holding nothing and touching
  nothing.
- No motor model, no belt stretch, no link flex. Rigid bodies and ideal joints.
- Straight-line arms only — one joint after another, like `kin`. No branching,
  no closed loops.
- `DYN_MAX_JOINTS` is 16 by default and sets the stack usage. Build with
  `-DDYN_MAX_JOINTS=6` on a small target.

## Files

```
include/dyn.h          the API, with comments
src/dyn.c              the implementation
examples/gravity.c     gravity compensation for a 2-link arm, then letting go
test/test_dyn.c        the tests, against closed-form textbook answers
```

## License

MIT. See [LICENSE](LICENSE).
