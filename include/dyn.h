/* dyn.h - inverse and forward dynamics for serial-chain arms.
 *
 * C99, no dependencies beyond libm, no malloc, no globals.
 * Same DH convention and joint types as kin, so a kin_link and a dyn_link
 * with the same four numbers describe the same arm.
 * Link with -lm.
 */
#ifndef DYN_H
#define DYN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Max joints in a chain. Override at build time: -DDYN_MAX_JOINTS=8 */
#ifndef DYN_MAX_JOINTS
#define DYN_MAX_JOINTS 16
#endif

/* Return codes. */
#define DYN_OK    0
#define DYN_EARG (-1)   /* bad argument, or a mass matrix that cannot be inverted */

typedef enum {
    DYN_REVOLUTE = 0,   /* q adds to theta */
    DYN_PRISMATIC = 1   /* q adds to d */
} dyn_joint_type;

/* One link: the same standard DH numbers kin uses, plus what it weighs.
 * T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
 *
 * The link frame's origin sits at the far end of the link, so a point mass
 * at the tip is com = {0, 0, 0}. */
typedef struct {
    double d, a, alpha, theta;
    dyn_joint_type type;
    double mass;          /* kg */
    double com[3];        /* centre of mass in this link's frame, m */
    double inertia[9];    /* 3x3 inertia about the centre of mass, in this link's frame, row-major */
} dyn_link;

typedef struct {
    const dyn_link *links;
    int n;                /* 1 .. DYN_MAX_JOINTS */
    double gravity[3];    /* in the base frame, m/s^2, e.g. {0, 0, -9.81}; all zero for no gravity */
} dyn_chain;

/* Inverse dynamics: the joint torques (N m) or forces (N) that produce qdd at q, qd.
 * qd and qdd may be NULL, meaning zero. Recursive Newton-Euler. */
int dyn_inverse(const dyn_chain *c, const double *q, const double *qd,
                const double *qdd, double *tau);

/* The torque that holds the arm still at q: dyn_inverse with qd = qdd = 0. */
int dyn_gravity(const dyn_chain *c, const double *q, double *tau);

/* Joint-space mass matrix M(q), n x n row-major, symmetric. */
int dyn_mass_matrix(const dyn_chain *c, const double *q, double *M);

/* Forward dynamics: the acceleration the arm has under tau at q, qd. Solves
 * M(q) qdd = tau - h(q, qd) where h holds gravity, Coriolis and centrifugal terms. */
int dyn_forward(const dyn_chain *c, const double *q, const double *qd,
                const double *tau, double *qdd);

#ifdef __cplusplus
}
#endif
#endif /* DYN_H */
