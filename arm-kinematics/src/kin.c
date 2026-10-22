#include "kin.h"
#include <math.h>
#include <string.h>

/* ---- small matrix helpers ------------------------------------------- */

static void mat4_identity(kin_mat4 m)
{
    memset(m, 0, 16 * sizeof(double));
    m[0] = m[5] = m[10] = m[15] = 1.0;
}

static void mat4_mul(const kin_mat4 a, const kin_mat4 b, kin_mat4 out)
{
    kin_mat4 t;
    int r, c, k;
    for (r = 0; r < 4; r++)
        for (c = 0; c < 4; c++) {
            double s = 0.0;
            for (k = 0; k < 4; k++) s += a[r * 4 + k] * b[k * 4 + c];
            t[r * 4 + c] = s;
        }
    memcpy(out, t, sizeof(t));
}

/* T_i = Rz(theta) Tz(d) Tx(a) Rx(alpha) */
static void dh_mat(const kin_link *l, double qi, kin_mat4 m)
{
    double th = l->theta + (l->type == KIN_REVOLUTE  ? qi : 0.0);
    double d  = l->d     + (l->type == KIN_PRISMATIC ? qi : 0.0);
    double ct = cos(th), st = sin(th);
    double ca = cos(l->alpha), sa = sin(l->alpha);

    m[0] = ct; m[1] = -st * ca; m[2]  =  st * sa; m[3]  = l->a * ct;
    m[4] = st; m[5] =  ct * ca; m[6]  = -ct * sa; m[7]  = l->a * st;
    m[8] = 0;  m[9] =       sa; m[10] =       ca; m[11] = d;
    m[12] = 0; m[13] = 0;       m[14] = 0;        m[15] = 1.0;
}

/* frames[0] = base, frames[i+1] = T_base->joint(i+1) */
static void fk_frames(const kin_chain *c, const double *q, kin_mat4 *frames)
{
    int i;
    mat4_identity(frames[0]);
    for (i = 0; i < c->n; i++) {
        kin_mat4 t;
        dh_mat(&c->links[i], q[i], t);
        mat4_mul(frames[i], t, frames[i + 1]);
    }
}

static int chain_ok(const kin_chain *c)
{
    return c && c->links && c->n >= 1 && c->n <= KIN_MAX_JOINTS;
}

/* ---- public: forward kinematics and Jacobian ---------------------------------------- */

int kin_forward(const kin_chain *c, const double *q, kin_mat4 out)
{
    kin_mat4 f[KIN_MAX_JOINTS + 1];
    if (!chain_ok(c) || !q || !out) return KIN_EARG;
    fk_frames(c, q, f);
    memcpy(out, f[c->n], sizeof(kin_mat4));
    return KIN_OK;
}

int kin_jacobian(const kin_chain *c, const double *q, double *J)
{
    kin_mat4 f[KIN_MAX_JOINTS + 1];
    double pe[3];
    int i, n;

    if (!chain_ok(c) || !q || !J) return KIN_EARG;
    n = c->n;
    fk_frames(c, q, f);
    pe[0] = f[n][3]; pe[1] = f[n][7]; pe[2] = f[n][11];

    for (i = 0; i < n; i++) {
        /* joint i acts about/along z of the frame before it */
        double zx = f[i][2],  zy = f[i][6],  zz = f[i][10];
        if (c->links[i].type == KIN_REVOLUTE) {
            double dx = pe[0] - f[i][3], dy = pe[1] - f[i][7], dz = pe[2] - f[i][11];
            J[0 * n + i] = zy * dz - zz * dy;
            J[1 * n + i] = zz * dx - zx * dz;
            J[2 * n + i] = zx * dy - zy * dx;
            J[3 * n + i] = zx; J[4 * n + i] = zy; J[5 * n + i] = zz;
        } else {
            J[0 * n + i] = zx; J[1 * n + i] = zy; J[2 * n + i] = zz;
            J[3 * n + i] = 0.0; J[4 * n + i] = 0.0; J[5 * n + i] = 0.0;
        }
    }
    return KIN_OK;
}

/* ---- inverse kinematics -------------------------------------------------------------- */

/* Rotation matrix (row-major 3x3) -> rotation vector (axis * angle).
 * Via quaternion so it stays well conditioned all the way to pi. */
static void rot_log(const double R[9], double w[3])
{
    double q[4], v, k;   /* q = w,x,y,z */
    double tr = R[0] + R[4] + R[8];

    if (tr > 0.0) {
        double s = sqrt(tr + 1.0) * 2.0;
        q[0] = 0.25 * s;
        q[1] = (R[7] - R[5]) / s;
        q[2] = (R[2] - R[6]) / s;
        q[3] = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        double s = sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
        q[0] = (R[7] - R[5]) / s;
        q[1] = 0.25 * s;
        q[2] = (R[1] + R[3]) / s;
        q[3] = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        double s = sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
        q[0] = (R[2] - R[6]) / s;
        q[1] = (R[1] + R[3]) / s;
        q[2] = 0.25 * s;
        q[3] = (R[5] + R[7]) / s;
    } else {
        double s = sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
        q[0] = (R[3] - R[1]) / s;
        q[1] = (R[2] + R[6]) / s;
        q[2] = (R[5] + R[7]) / s;
        q[3] = 0.25 * s;
    }
    if (q[0] < 0.0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3]; }

    v = sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    k = (v < 1e-12) ? 2.0 : 2.0 * atan2(v, q[0]) / v;
    w[0] = k * q[1]; w[1] = k * q[2]; w[2] = k * q[3];
}

/* e = [target - current] as position error + rotation vector */
static void pose_err(const kin_mat4 cur, const kin_mat4 tgt, double e[6])
{
    double R[9], w[3];
    int r, c, k;

    e[0] = tgt[3] - cur[3];
    e[1] = tgt[7] - cur[7];
    e[2] = tgt[11] - cur[11];

    /* R = R_tgt * R_cur^T */
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++) {
            double s = 0.0;
            for (k = 0; k < 3; k++) s += tgt[r * 4 + k] * cur[c * 4 + k];
            R[r * 3 + c] = s;
        }
    rot_log(R, w);
    e[3] = w[0]; e[4] = w[1]; e[5] = w[2];
}

/* Gauss elimination with partial pivoting, 6x6. A and b are destroyed. */
static int solve6(double A[36], double b[6], double x[6])
{
    int i, j, k;
    for (k = 0; k < 6; k++) {
        int piv = k;
        double max = fabs(A[k * 6 + k]);
        for (i = k + 1; i < 6; i++)
            if (fabs(A[i * 6 + k]) > max) { max = fabs(A[i * 6 + k]); piv = i; }
        if (max < 1e-15) return KIN_EARG;          /* singular even with damping */
        if (piv != k) {
            for (j = 0; j < 6; j++) {
                double t = A[k * 6 + j]; A[k * 6 + j] = A[piv * 6 + j]; A[piv * 6 + j] = t;
            }
            { double t = b[k]; b[k] = b[piv]; b[piv] = t; }
        }
        for (i = k + 1; i < 6; i++) {
            double f = A[i * 6 + k] / A[k * 6 + k];
            if (f == 0.0) continue;
            for (j = k; j < 6; j++) A[i * 6 + j] -= f * A[k * 6 + j];
            b[i] -= f * b[k];
        }
    }
    for (i = 5; i >= 0; i--) {
        double s = b[i];
        for (j = i + 1; j < 6; j++) s -= A[i * 6 + j] * x[j];
        x[i] = s / A[i * 6 + i];
    }
    return KIN_OK;
}

int kin_inverse(const kin_chain *c, const kin_mat4 target,
           const double *q0, double *q_out, const kin_inverse_opts *opts)
{
    double pos_tol, rot_tol, lambda, max_step;
    int max_iters, n, i, j, k, iter;

    if (!chain_ok(c) || !target || !q0 || !q_out) return KIN_EARG;
    n = c->n;

    pos_tol   = (opts && opts->pos_tol   > 0.0) ? opts->pos_tol   : 1e-4;
    rot_tol   = (opts && opts->rot_tol   > 0.0) ? opts->rot_tol   : 1e-4;
    lambda    = (opts && opts->lambda    > 0.0) ? opts->lambda    : 0.05;
    max_step  = (opts && opts->max_step  > 0.0) ? opts->max_step  : 0.2;
    max_iters = (opts && opts->max_iters > 0)   ? opts->max_iters : 100;

    memmove(q_out, q0, (size_t)n * sizeof(double));

    for (iter = 0; iter < max_iters; iter++) {
        kin_mat4 cur;
        double J[6 * KIN_MAX_JOINTS];
        double A[36], e[6], y[6], dq[KIN_MAX_JOINTS];
        double pe, re, norm;

        kin_forward(c, q_out, cur);
        pose_err(cur, target, e);

        pe = sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        re = sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);
        if (pe < pos_tol && re < rot_tol) return KIN_OK;

        kin_jacobian(c, q_out, J);

        /* A = J J^T + lambda^2 I */
        for (i = 0; i < 6; i++)
            for (j = 0; j < 6; j++) {
                double s = 0.0;
                for (k = 0; k < n; k++) s += J[i * n + k] * J[j * n + k];
                A[i * 6 + j] = s + (i == j ? lambda * lambda : 0.0);
            }

        if (solve6(A, e, y) != KIN_OK) return KIN_ENOCONV;

        /* dq = J^T y, then limit the step so the linearisation stays valid */
        norm = 0.0;
        for (j = 0; j < n; j++) {
            double s = 0.0;
            for (i = 0; i < 6; i++) s += J[i * n + j] * y[i];
            dq[j] = s;
            norm += s * s;
        }
        norm = sqrt(norm);
        if (norm > max_step) {
            double sc = max_step / norm;
            for (j = 0; j < n; j++) dq[j] *= sc;
        }

        for (j = 0; j < n; j++) {
            const kin_link *l = &c->links[j];
            q_out[j] += dq[j];
            if (l->qmin < l->qmax) {                 /* both 0 => no limit */
                if (q_out[j] < l->qmin) q_out[j] = l->qmin;
                if (q_out[j] > l->qmax) q_out[j] = l->qmax;
            }
        }
    }
    return KIN_ENOCONV;
}

/* ---- pose helpers ---------------------------------------------------- */

void kin_pose(double x, double y, double z,
              double roll, double pitch, double yaw, kin_mat4 out)
{
    double cr = cos(roll),  sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    out[0] = cy * cp; out[1] = cy * sp * sr - sy * cr; out[2]  = cy * sp * cr + sy * sr; out[3]  = x;
    out[4] = sy * cp; out[5] = sy * sp * sr + cy * cr; out[6]  = sy * sp * cr - cy * sr; out[7]  = y;
    out[8] = -sp;     out[9] = cp * sr;                out[10] = cp * cr;                out[11] = z;
    out[12] = 0; out[13] = 0; out[14] = 0; out[15] = 1.0;
}

void kin_pose_decompose(const kin_mat4 t, double xyzrpy[6])
{
    double sp = -t[8];
    if (sp > 1.0) sp = 1.0;
    if (sp < -1.0) sp = -1.0;
    xyzrpy[0] = t[3];
    xyzrpy[1] = t[7];
    xyzrpy[2] = t[11];
    xyzrpy[3] = atan2(t[9], t[10]);                       /* roll  */
    xyzrpy[4] = asin(sp);                                 /* pitch */
    xyzrpy[5] = atan2(t[4], t[0]);                        /* yaw   */
}
