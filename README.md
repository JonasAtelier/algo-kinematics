# kin

Portable forward and inverse kinematics for serial chains, in one C file.

- C99, only `libm` on top of the standard library
- No `malloc`, no globals, no I/O — safe on MCUs
- One header, one source file: drop `include/kin.h` + `src/kin.c` into any project
- Forward by Denavit–Hartenberg, inverse by damped least squares (works for any chain, no
  robot-specific solver to write)

## Build

```sh
make          # libkin.a + example + tests
make check    # run the self-check
make run      # run the 6-axis example
```

Or just compile `src/kin.c` with your own build. Cap the stack usage of a small
target with `-DKIN_MAX_JOINTS=6`.

## The API

Six functions, three types. That is all of it.

### Describe the robot

```c
kin_link links[] = {
    /*  d      a      alpha   theta  type          qmin   qmax  */
    { 0.089,   0.0,   M_PI/2, 0.0,   KIN_REVOLUTE, -6.28, 6.28 },
    { 0.0,    -0.425, 0.0,    0.0,   KIN_REVOLUTE, -6.28, 6.28 },
    /* ... */
};
kin_chain arm = { links, 2 };
```

Standard (distal) DH: `T = Rz(theta) Tz(d) Tx(a) Rx(alpha)`. A `KIN_REVOLUTE`
joint adds `q` to `theta`, a `KIN_PRISMATIC` joint adds `q` to `d`. Leave
`qmin`/`qmax` both `0` for an unlimited joint.

### Forward kinematics

```c
kin_mat4 pose;               /* double[16], row-major 4x4 */
kin_forward(&arm, q, pose);       /* q has arm.n entries */
```

### Inverse kinematics

```c
double sol[6];
int rc = kin_inverse(&arm, target, seed, sol, NULL);   /* NULL = default tuning */
```

`seed` is the starting guess — usually the robot's current joint values. Returns
`KIN_OK`, or `KIN_ENOCONV` if it ran out of iterations (`sol` still holds the
closest iterate it found). Joint limits are respected.

Tune it when the default is not right for your arm:

```c
kin_inverse_opts o = { 0 };       /* every field left 0 keeps its default */
o.max_iters = 300;
o.lambda    = 0.1;           /* more damping: slower, but stops oscillation */
kin_inverse(&arm, target, seed, sol, &o);
```

| field       | default | meaning                        |
|-------------|---------|--------------------------------|
| `pos_tol`   | 1e-4    | position tolerance, metres     |
| `rot_tol`   | 1e-4    | orientation tolerance, radians |
| `lambda`    | 0.05    | damping near singularities     |
| `max_step`  | 0.2     | max joint step per iteration   |
| `max_iters` | 100     | iteration cap                  |

### Poses

`kin_mat4` is a plain `double[16]`, row-major, so `t[3]`, `t[7]`, `t[11]` is the
position. Two helpers save you writing rotation matrices by hand:

```c
kin_mat4 target;
kin_pose(0.4, 0.1, 0.3,  0.0, 1.57, 0.0, target);   /* x y z roll pitch yaw */

double p[6];
kin_pose_decompose(target, p);                      /* -> x y z roll pitch yaw */
```

Rotation order is `R = Rz(yaw) Ry(pitch) Rx(roll)`.

### Jacobian

Exposed because it is free — `kin_inverse` computes it anyway. 6 rows × `n` columns,
row-major; rows 0–2 linear, rows 3–5 angular. Useful for velocity control or
your own solver.

```c
double J[6 * 6];
kin_jacobian(&arm, q, J);
```

## Layout

```
include/kin.h        the whole public API
src/kin.c            the whole implementation
examples/arm6.c      UR5: forward to a pose, then inverse back to it
test/test_kin.c      assert-based self-check
```

## Scope

Serial chains only. No trees, no branches, no closed loops, no dynamics, no
trajectory planning. Inverse is numeric and local: it finds the solution nearest the
seed, not all eight of them. That is deliberate — it is what most arms need, and
it fits in one file.
