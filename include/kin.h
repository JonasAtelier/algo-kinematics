/* kin.h - portable forward & inverse kinematics for serial chains.
 *
 * C99, no dependencies beyond libm, no malloc, no globals.
 * Link with -lm.
 */
#ifndef KIN_H
#define KIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Max joints in a chain. Override at build time: -DKIN_MAX_JOINTS=8 */
#ifndef KIN_MAX_JOINTS
#define KIN_MAX_JOINTS 16
#endif

/* Return codes. */
#define KIN_OK       0
#define KIN_EARG    (-1)   /* bad argument (NULL, n out of range) */
#define KIN_ENOCONV (-2)   /* inverse hit max_iters without reaching tolerance */

/* A 4x4 homogeneous transform, row-major: t[row*4 + col]. */
typedef double kin_mat4[16];

typedef enum {
    KIN_REVOLUTE = 0,   /* q adds to theta */
    KIN_PRISMATIC = 1   /* q adds to d */
} kin_joint_type;

/* One link, standard (distal) Denavit-Hartenberg parameters.
 * T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
 *
 * qmin/qmax: joint limits, in rad or m. Leave both 0 for no limit. */
typedef struct {
    double d, a, alpha, theta;
    kin_joint_type type;
    double qmin, qmax;
} kin_link;

typedef struct {
    const kin_link *links;
    int n;              /* 1 .. KIN_MAX_JOINTS */
} kin_chain;

/* Inverse-kinematics tuning. Any field left 0 uses its default. */
typedef struct {
    double pos_tol;     /* default 1e-4  (metres) */
    double rot_tol;     /* default 1e-4  (radians) */
    double lambda;      /* default 0.05  damping; raise if it oscillates */
    double max_step;    /* default 0.2   max |dq| per iteration */
    int    max_iters;   /* default 100 */
} kin_inverse_opts;

/* Forward kinematics: pose of the tip in base frame. q has c->n entries. */
int kin_forward(const kin_chain *c, const double *q, kin_mat4 out);

/* Geometric Jacobian at q, 6 rows x c->n cols, row-major: J[row*n + col].
 * Rows 0-2 linear, rows 3-5 angular. J must hold 6*n doubles. */
int kin_jacobian(const kin_chain *c, const double *q, double *J);

/* Inverse kinematics: damped least squares from seed q0.
 * Writes c->n joint values to q_out (q_out may alias q0).
 * opts may be NULL. Returns KIN_OK, KIN_ENOCONV or KIN_EARG.
 * On KIN_ENOCONV q_out still holds the best iterate reached. */
int kin_inverse(const kin_chain *c, const kin_mat4 target,
                const double *q0, double *q_out, const kin_inverse_opts *opts);

/* Pose helpers. Rotation is R = Rz(yaw) * Ry(pitch) * Rx(roll). */
void kin_pose(double x, double y, double z,
              double roll, double pitch, double yaw, kin_mat4 out);
void kin_pose_decompose(const kin_mat4 t, double xyzrpy[6]);

#ifdef __cplusplus
}
#endif
#endif /* KIN_H */
