# mob

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: GPL-3.0-only" src="https://img.shields.io/badge/license-GPL--3.0--only-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Wheel kinematics and dead-reckoning odometry for mobile bases, in one C file.

Two questions, and this library answers both:

- **Inverse** — I want the robot to move *like this*, so what do the wheels do?
- **Forward** — the wheels did *this*, so how did the robot move?

Answer them for a differential drive, a mecanum base or a car, then add the
answers up over time to keep track of where the robot thinks it is.

- One header, one source file. Drop them into any C project.
- Only needs `libm`. No `malloc` — works fine on a microcontroller.
- Three base types, one API. No robot-specific maths to derive.

## Try it

```sh
make check    # run the tests
make run      # walk a rover round a 1 m square
```

To use it in your own project, copy `include/mob.h` and `src/mob.c`, link the
CMake target `algo::wheel-odometry`, or use this folder as an ESP-IDF
component. See [Use one library](../README.md#use-one-library).

## The three bases

You describe a base with a type, a wheel radius and up to two dimensions. Which
dimensions matter depends on the type; the ones a type does not use are ignored.

### Differential drive

Two driven wheels on one axle, plus a castor or two to stop it falling over.
It can go forward and it can turn. It cannot go sideways.

```
              front
        ,---------------,
   [0] =|               |= [1]
        |       +       |
        |               |
        '-------o-------'
        |<--- track --->|
```

`[0]` left wheel, `[1]` right wheel. `+` is the point the pose tracks, `o` is
the castor. Only `track` matters, the distance between the two wheels.

```c
mob_base b = { MOB_DIFF, 0.05, 0.40, 0.0 };   /* 5 cm wheels, 40 cm apart */
```

### Mecanum

Four wheels with rollers set at 45 degrees, in the usual X layout. Spin them in
the right combination and it goes sideways, or diagonally, or turns while doing
either. This is the one that can crab.

```
              front
   [0] =,---------------,= [1]     ---
        |               |           ^
        |       +       |           | wheelbase
        |               |           v
   [2] ='---------------'= [3]     ---
        |<--- track --->|
```

`[0]` front-left, `[1]` front-right, `[2]` rear-left, `[3]` rear-right. Both
`track` and `wheelbase` matter, since a mecanum base turns about its centre and
each wheel's lever arm depends on both.

```c
mob_base b = { MOB_MECANUM, 0.05, 0.40, 0.30 };
```

Get the roller direction wrong on one corner and the robot will fight itself.
`[0]` and `[3]` have their rollers on one diagonal, `[1]` and `[2]` on the other.

### Ackermann

A car. The rear axle drives, the front wheels steer, and it is treated as a
bicycle: one drive speed and one steer angle.

```
              front
   [1] \,---------------,/ [1]     ---
        |               |           ^
        |       +       |           | wheelbase
        |               |           v
   [0] ='---------------'= [0]     ---
```

`wheels[0]` is the rear axle speed in rad/s and `wheels[1]` is the steer angle
in radians — not two wheels, two different numbers. Only `wheelbase` matters,
the distance from the rear axle to the front.

```c
mob_base b = { MOB_ACKERMANN, 0.30, 0.0, 2.50 };
```

A car cannot slide sideways, so a demanded `vy` is ignored. A car cannot turn on
the spot either, so asking for `vx == 0` with `wz != 0` gives `MOB_EARG`. Asking
for `vx == 0` with `wz == 0` is just "stop", and works.

## Units and frames

Everything is metres, radians and seconds. The body frame sits at the `+` in the
drawings, x out of the nose and y out of the left side.

| name | meaning | unit |
|------|---------|------|
| `vx` | forward, out of the nose | m/s |
| `vy` | left, out of the left side | m/s |
| `wz` | turning, counter-clockwise seen from above | rad/s |
| `x`, `y` | where it is in the world | m |
| `theta` | which way it points, `0` is along world +x | rad |
| `wheels[i]` | how fast that wheel spins | rad/s |
| `wheels[1]` | Ackermann only: the steer angle | rad |

`theta` always comes back wrapped to `(-pi, pi]`.

## The functions

```c
int  mob_wheels(&base);                    /* 2, 4 or 2 */
int  mob_inverse(&base, &twist, wheels);   /* how I want to move -> what the wheels do */
int  mob_forward(&base, wheels, &twist);   /* what the wheels did -> how I moved */
void mob_integrate(&pose, &twist, dt);     /* add one step onto a world pose */
```

| call | what it does |
|------|--------------|
| `mob_wheels` | how many `wheels[]` entries this base uses, so you size the array right |
| `mob_inverse` | a body velocity becomes wheel commands you can send to the motors |
| `mob_forward` | wheel speeds off the encoders become the velocity the body actually had |
| `mob_integrate` | advances a pose by one time step; exact arc when turning |

`mob_inverse` and `mob_forward` return `MOB_OK` or `MOB_EARG`. `MOB_EARG` means
a `NULL` pointer, a number that is not finite, a wheel radius or dimension that
is zero or negative, or a manoeuvre the base physically cannot do.
`mob_integrate` has no return code — bad arguments leave the pose untouched.

A whole control loop is three calls:

```c
mob_base  base = { MOB_DIFF, 0.05, 0.40, 0.0 };
mob_pose  pose = { 0.0, 0.0, 0.0 };
mob_twist want = { 0.5, 0.0, 0.2 };        /* forward and curving left */
double    w[MOB_MAX_WHEELS];

mob_inverse(&base, &want, w);              /* send w[0], w[1] to the motors */

mob_twist got;
mob_forward(&base, measured, &got);        /* measured: what the encoders read */
mob_integrate(&pose, &got, dt);            /* pose now says where it thinks it is */
```

## Odometry drifts

Dead reckoning adds up every step and never subtracts anything, so every small
error stays in the answer forever. Wheels slip on a turn, the real wheel radius
is never quite the number on the datasheet, the track is measured with a tape
measure, and the encoder rounds. Each step is off by a hair, and the hairs add.
Heading is the one that hurts: once `theta` is wrong, every later step is driven
off in the wrong direction, so position error grows faster than the distance
travelled. Left alone, a robot crossing a room is out by centimetres and one
crossing a warehouse is out by metres. The fix is not a better integrator, it is
a second opinion from something that measures the world instead of the wheels —
a gyro fused with
[`f_madgwick`](https://github.com/JonasAtelier/algo-filter/tree/stable/madgwick)
to hold the heading, or lidar scan matching to correct the position. This
library gives you the estimate to correct, not the correction.

## What it does not do

- No planning and no obstacle avoidance. It answers where you are and what the
  wheels should do, not where to go.
- No slip model. It believes the encoders completely.
- No dynamics — no masses, no forces, no traction limits. Ask for an impossible
  acceleration and it will hand you the wheel speeds without comment.
- No motor limits or acceleration ramps. Clamp the output yourself.
- Three base types only. No omni-wheels at 120 degrees, no swerve, no tracks,
  no legs.
- No sensor fusion and no filtering. See "odometry drifts" above.

## License

© 2026 JonasAtelier, GPL-3.0-only. See [LICENSE](LICENSE).
