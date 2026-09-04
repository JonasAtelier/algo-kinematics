/* dyn.c - recursive Newton-Euler dynamics for serial chains.
 *
 * The recursion is the classic one (Luh, Walker & Paul; Craig ch. 6): an
 * outward pass for how each link frame moves, an inward pass for the force
 * and moment that motion takes. Everything is expressed in the link's own
 * frame, so no 4x4 ever appears -- one rotation and one offset per link is
 * the whole geometry.
 */
#include "dyn.h"
#include <math.h>
#include <string.h>

/* ---- small vector helpers -------------------------------------------- */

static void v3_cross(const double a[3], const double b[3], double out[3])
{
    double x = a[1] * b[2] - a[2] * b[1];
    double y = a[2] * b[0] - a[0] * b[2];
    double z = a[0] * b[1] - a[1] * b[0];
    out[0] = x; out[1] = y; out[2] = z;
}

static double v3_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* out = M v, M row-major 3x3 */
static void m3_mul(const double M[9], const double v[3], double out[3])
{
    double x = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
    double y = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
    double z = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
    out[0] = x; out[1] = y; out[2] = z;
}

/* out = M^T v: with M the link rotation, this drags a vector from the
 * previous link's frame into this one. */
static void m3t_mul(const double M[9], const double v[3], double out[3])
{
    double x = M[0] * v[0] + M[3] * v[1] + M[6] * v[2];
    double y = M[1] * v[0] + M[4] * v[1] + M[7] * v[2];
    double z = M[2] * v[0] + M[5] * v[1] + M[8] * v[2];
    out[0] = x; out[1] = y; out[2] = z;
}

/* Rotation and offset of T_i = Rz(theta) Tz(d) Tx(a) Rx(alpha), kin's
 * convention exactly.  R is ^(i-1)R_i, row-major 3x3.  pstar is the vector
 * from origin i-1 to origin i, already expressed in frame i:
 *   pstar = R^T (a cos th, a sin th, d) = (a, d sin alpha, d cos alpha).
 * Row 2 of R is the joint axis z_(i-1) seen from frame i, (0, sa, ca). */
static void dh_frame(const dyn_link *l, double qi, double R[9], double pstar[3])
{
    double th = l->theta + (l->type == DYN_REVOLUTE  ? qi : 0.0);
    double d  = l->d     + (l->type == DYN_PRISMATIC ? qi : 0.0);
    double ct = cos(th), st = sin(th);
    double ca = cos(l->alpha), sa = sin(l->alpha);

    R[0] = ct; R[1] = -st * ca; R[2] =  st * sa;
    R[3] = st; R[4] =  ct * ca; R[5] = -ct * sa;
    R[6] = 0;  R[7] =       sa; R[8] =       ca;

    pstar[0] = l->a; pstar[1] = d * sa; pstar[2] = d * ca;
}

/* ---- the engine ------------------------------------------------------- */

/* Recursive Newton-Euler.  qd and qdd NULL mean zero.  use_gravity 0 leaves
 * the base still instead of accelerating it, which is what the mass matrix
 * wants.  Assumes the arguments have already been validated. */
static void rne(const dyn_chain *c, const double *q, const double *qd,
                const double *qdd, int use_gravity, double *tau)
{
    double R[DYN_MAX_JOINTS][9], pstar[DYN_MAX_JOINTS][3];
    double F[DYN_MAX_JOINTS][3], N[DYN_MAX_JOINTS][3];
    double w[3] = { 0.0, 0.0, 0.0 };        /* angular velocity of frame i   */
    double wd[3] = { 0.0, 0.0, 0.0 };       /* its angular acceleration      */
    double a[3];                            /* acceleration of origin i      */
    double f[3] = { 0.0, 0.0, 0.0 };        /* force through joint i+1       */
    double nn[3] = { 0.0, 0.0, 0.0 };       /* moment through joint i+1      */
    int i, k;

    /* Gravity enters as the base accelerating upwards at g: an arm holding
     * itself up in gravity does the same work as one being hoisted at g. */
    for (k = 0; k < 3; k++) a[k] = use_gravity ? -c->gravity[k] : 0.0;

    /* outward: base to tip, working out how every link moves */
    for (i = 0; i < c->n; i++) {
        const dyn_link *l = &c->links[i];
        double v = qd ? qd[i] : 0.0, acc = qdd ? qdd[i] : 0.0;
        double wp[3], wdp[3], ap[3], zq[3], z[3], t1[3], t2[3], ac[3];

        memcpy(wp, w, sizeof(wp));
        memcpy(wdp, wd, sizeof(wdp));
        memcpy(ap, a, sizeof(ap));

        dh_frame(l, q[i], R[i], pstar[i]);
        z[0] = R[i][6]; z[1] = R[i][7]; z[2] = R[i][8];   /* joint axis, in frame i */

        if (l->type == DYN_REVOLUTE) {
            zq[0] = 0.0; zq[1] = 0.0; zq[2] = v;          /* z * qd, in frame i-1 */
            v3_cross(wp, zq, t1);                          /* the gyroscopic term  */
            for (k = 0; k < 3; k++) t1[k] += wdp[k];
            t1[2] += acc;
            m3t_mul(R[i], t1, wd);
            for (k = 0; k < 3; k++) t1[k] = wp[k] + zq[k];
            m3t_mul(R[i], t1, w);
            m3t_mul(R[i], ap, a);
        } else {
            m3t_mul(R[i], wp, w);                          /* sliding adds no spin */
            m3t_mul(R[i], wdp, wd);
            m3t_mul(R[i], ap, a);
            for (k = 0; k < 3; k++) a[k] += acc * z[k];    /* the slide itself     */
            v3_cross(w, z, t1);                            /* Coriolis             */
            for (k = 0; k < 3; k++) a[k] += 2.0 * v * t1[k];
        }

        /* carry the origin's acceleration across the link */
        v3_cross(wd, pstar[i], t1);
        v3_cross(w, pstar[i], t2);
        v3_cross(w, t2, t2);
        for (k = 0; k < 3; k++) a[k] += t1[k] + t2[k];

        /* and out to the centre of mass */
        v3_cross(wd, l->com, t1);
        v3_cross(w, l->com, t2);
        v3_cross(w, t2, t2);
        for (k = 0; k < 3; k++) ac[k] = a[k] + t1[k] + t2[k];

        for (k = 0; k < 3; k++) F[i][k] = l->mass * ac[k];

        m3_mul(l->inertia, wd, t1);                        /* I wd            */
        m3_mul(l->inertia, w, t2);
        v3_cross(w, t2, t2);                               /* w x (I w)       */
        for (k = 0; k < 3; k++) N[i][k] = t1[k] + t2[k];
    }

    /* inward: tip to base, adding up what each joint has to hold.  f and nn
     * are the wrench link i-1 puts on link i, taken about origin i-1, which
     * is the point the joint axis runs through. */
    for (i = c->n - 1; i >= 0; i--) {
        const dyn_link *l = &c->links[i];
        double fn[3], nnn[3], arm[3], t1[3], t2[3], z[3];

        if (i + 1 < c->n) {
            m3_mul(R[i + 1], f, fn);                       /* drag the child's */
            m3_mul(R[i + 1], nn, nnn);                     /* wrench back      */
        } else {
            fn[0] = fn[1] = fn[2] = 0.0;                   /* free tip         */
            nnn[0] = nnn[1] = nnn[2] = 0.0;
        }

        v3_cross(pstar[i], fn, t1);
        for (k = 0; k < 3; k++) arm[k] = pstar[i][k] + l->com[k];
        v3_cross(arm, F[i], t2);
        for (k = 0; k < 3; k++) {
            nn[k] = nnn[k] + t1[k] + t2[k] + N[i][k];
            f[k]  = fn[k] + F[i][k];
        }

        z[0] = R[i][6]; z[1] = R[i][7]; z[2] = R[i][8];
        tau[i] = (l->type == DYN_REVOLUTE) ? v3_dot(nn, z) : v3_dot(f, z);
    }
}

/* ---- validation ------------------------------------------------------- */

static int finite_arr(const double *v, int n)
{
    int i;
    for (i = 0; i < n; i++) if (!isfinite(v[i])) return 0;
    return 1;
}

static int chain_ok(const dyn_chain *c)
{
    int i;
    if (!c || !c->links || c->n < 1 || c->n > DYN_MAX_JOINTS) return 0;
    if (!finite_arr(c->gravity, 3)) return 0;
    for (i = 0; i < c->n; i++) {
        const dyn_link *l = &c->links[i];
        if (!isfinite(l->d) || !isfinite(l->a) ||
            !isfinite(l->alpha) || !isfinite(l->theta)) return 0;
        if (!isfinite(l->mass) || l->mass < 0.0) return 0;
        if (!finite_arr(l->com, 3) || !finite_arr(l->inertia, 9)) return 0;
    }
    return 1;
}

/* ---- public ----------------------------------------------------------- */

int dyn_inverse(const dyn_chain *c, const double *q, const double *qd,
                const double *qdd, double *tau)
{
    if (!chain_ok(c) || !q || !tau) return DYN_EARG;
    if (!finite_arr(q, c->n)) return DYN_EARG;
    if (qd && !finite_arr(qd, c->n)) return DYN_EARG;
    if (qdd && !finite_arr(qdd, c->n)) return DYN_EARG;
    rne(c, q, qd, qdd, 1, tau);
    return DYN_OK;
}

int dyn_gravity(const dyn_chain *c, const double *q, double *tau)
{
    return dyn_inverse(c, q, NULL, NULL, tau);
}

int dyn_mass_matrix(const dyn_chain *c, const double *q, double *M)
{
    double e[DYN_MAX_JOINTS] = { 0.0 }, col[DYN_MAX_JOINTS];
    int i, j, n;

    if (!chain_ok(c) || !q || !M) return DYN_EARG;
    if (!finite_arr(q, c->n)) return DYN_EARG;
    n = c->n;

    /* Column j is the torque a unit acceleration of joint j alone takes, with
     * gravity and velocity switched off -- which is the definition of M.
     * ponytail: O(n) RNE called n times for M; a composite-rigid-body pass is
     * faster if n > 8 matters */
    for (j = 0; j < n; j++) {
        e[j] = 1.0;
        rne(c, q, NULL, e, 0, col);
        e[j] = 0.0;
        for (i = 0; i < n; i++) M[i * n + j] = col[i];
    }

    /* M is symmetric in theory; make it symmetric in floating point too */
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++) {
            double m = 0.5 * (M[i * n + j] + M[j * n + i]);
            M[i * n + j] = M[j * n + i] = m;
        }
    return DYN_OK;
}

/* Gauss elimination with partial pivoting, n x n row-major. A and b destroyed. */
static int solve_n(double *A, double *b, double *x, int n)
{
    int i, j, k;

    for (k = 0; k < n; k++) {
        int piv = k;
        double max = fabs(A[k * n + k]);
        for (i = k + 1; i < n; i++)
            if (fabs(A[i * n + k]) > max) { max = fabs(A[i * n + k]); piv = i; }
        if (max < 1e-12) return DYN_EARG;      /* a massless or degenerate arm */
        if (piv != k) {
            for (j = 0; j < n; j++) {
                double t = A[k * n + j]; A[k * n + j] = A[piv * n + j]; A[piv * n + j] = t;
            }
            { double t = b[k]; b[k] = b[piv]; b[piv] = t; }
        }
        for (i = k + 1; i < n; i++) {
            double fac = A[i * n + k] / A[k * n + k];
            if (fac == 0.0) continue;
            for (j = k; j < n; j++) A[i * n + j] -= fac * A[k * n + j];
            b[i] -= fac * b[k];
        }
    }
    for (i = n - 1; i >= 0; i--) {
        double s = b[i];
        for (j = i + 1; j < n; j++) s -= A[i * n + j] * x[j];
        x[i] = s / A[i * n + i];
    }
    return DYN_OK;
}

int dyn_forward(const dyn_chain *c, const double *q, const double *qd,
                const double *tau, double *qdd)
{
    double M[DYN_MAX_JOINTS * DYN_MAX_JOINTS];
    double h[DYN_MAX_JOINTS], b[DYN_MAX_JOINTS];
    int i, n;

    if (!chain_ok(c) || !q || !tau || !qdd) return DYN_EARG;
    if (!finite_arr(q, c->n) || !finite_arr(tau, c->n)) return DYN_EARG;
    if (qd && !finite_arr(qd, c->n)) return DYN_EARG;
    n = c->n;

    rne(c, q, qd, NULL, 1, h);                 /* gravity, Coriolis, centrifugal */
    if (dyn_mass_matrix(c, q, M) != DYN_OK) return DYN_EARG;
    for (i = 0; i < n; i++) b[i] = tau[i] - h[i];
    return solve_n(M, b, qdd, n);
}
