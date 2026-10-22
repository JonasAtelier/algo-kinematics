/* Self-check: assert-based, no framework. Build with `make check`. */
#include "dyn.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
#define G  9.81
#define NEAR(a, b, tol) (fabs((a) - (b)) < (tol))

#define NO_INERTIA { 0,0,0, 0,0,0, 0,0,0 }

/* ---- 1. one revolute joint, a point mass l out along the link ------------
 *
 * The same arm, written the two ways the DH frame allows: the mass as a com
 * offset from a zero-length link, or sitting on the origin of a link of
 * length l (a link frame's origin is at the far end of the link). Gravity
 * along -y, so the arm swings in the x-y plane and holding it takes
 * tau = m g l cos(q) -- positive at q = 0, zero straight up. */

static void test_point_mass_pendulum(void)
{
    static const double qs[5] = { 0.0, 0.5, -1.2, PI / 2, 2.5 };
    static const dyn_link com_offset[1] = {
        /*  d    a    alpha theta  type          mass  com                inertia */
        {  0.0, 0.0,  0.0,  0.0,   DYN_REVOLUTE, 2.5,  { 0.4, 0.0, 0.0 }, NO_INERTIA },
    };
    static const dyn_link tip_of_link[1] = {
        {  0.0, 0.4,  0.0,  0.0,   DYN_REVOLUTE, 2.5,  { 0.0, 0.0, 0.0 }, NO_INERTIA },
    };
    dyn_chain a = { com_offset,  1, { 0.0, -G, 0.0 } };
    dyn_chain b = { tip_of_link, 1, { 0.0, -G, 0.0 } };
    int i;

    for (i = 0; i < 5; i++) {
        double ta, tb, want = 2.5 * G * 0.4 * cos(qs[i]);
        assert(dyn_gravity(&a, &qs[i], &ta) == DYN_OK);
        assert(dyn_gravity(&b, &qs[i], &tb) == DYN_OK);
        assert(NEAR(ta, want, 1e-9));
        assert(NEAR(tb, want, 1e-9));
    }
}

/* ---- planar 2R, point masses at the two link tips ----------------------- */

#define L1 0.5
#define L2 0.4
#define M1 2.0
#define M2 1.0

static const dyn_link planar2[2] = {
    /*  d    a   alpha theta  type          mass  com                inertia */
    {  0.0, L1,  0.0,  0.0,   DYN_REVOLUTE, M1,   { 0.0, 0.0, 0.0 }, NO_INERTIA },
    {  0.0, L2,  0.0,  0.0,   DYN_REVOLUTE, M2,   { 0.0, 0.0, 0.0 }, NO_INERTIA },
};

/* ---- 2. gravity torques against the textbook closed form ---------------- */

static void test_planar_gravity(void)
{
    static const double poses[3][2] = { { 0.0, 0.0 }, { 0.6, -0.9 }, { -1.3, 2.0 } };
    dyn_chain c = { planar2, 2, { 0.0, -G, 0.0 } };
    int i;

    for (i = 0; i < 3; i++) {
        double q1 = poses[i][0], q2 = poses[i][1], tau[2];
        double want1 = (M1 + M2) * G * L1 * cos(q1) + M2 * G * L2 * cos(q1 + q2);
        double want2 = M2 * G * L2 * cos(q1 + q2);

        assert(dyn_gravity(&c, poses[i], tau) == DYN_OK);
        assert(NEAR(tau[0], want1, 1e-9));
        assert(NEAR(tau[1], want2, 1e-9));
    }
}

/* ---- 3. mass matrix against the textbook closed form -------------------- */

static void test_planar_mass_matrix(void)
{
    static const double poses[3][2] = { { 0.0, 0.0 }, { 0.6, -0.9 }, { -1.3, 2.0 } };
    dyn_chain c = { planar2, 2, { 0.0, -G, 0.0 } };
    int i;

    for (i = 0; i < 3; i++) {
        double q2 = poses[i][1], M[4];
        double m11 = M1 * L1 * L1 + M2 * (L1 * L1 + L2 * L2 + 2 * L1 * L2 * cos(q2));
        double m12 = M2 * (L2 * L2 + L1 * L2 * cos(q2));
        double m22 = M2 * L2 * L2;

        assert(dyn_mass_matrix(&c, poses[i], M) == DYN_OK);
        assert(NEAR(M[1], M[2], 1e-9));          /* symmetric */
        assert(NEAR(M[0], m11, 1e-9));
        assert(NEAR(M[1], m12, 1e-9));
        assert(NEAR(M[3], m22, 1e-9));
    }
}

/* ---- a spatial arm: twisted axes, real inertia tensors, one prismatic ---- */

static const dyn_link spatial3[3] = {
    /*  d     a     alpha    theta type           mass  com                   inertia */
    { 0.15,  0.0,   PI / 2,  0.0,  DYN_REVOLUTE,  3.0,  { 0.0, -0.05, 0.02 },
      { 0.020,0,0, 0,0.025,0, 0,0,0.010 } },
    { 0.0,   0.35,  0.0,     0.0,  DYN_REVOLUTE,  2.0,  { -0.15, 0.0, 0.03 },
      { 0.010,0,0, 0,0.020,0, 0,0,0.015 } },
    { 0.10,  0.0,  -PI / 2,  0.0,  DYN_PRISMATIC, 1.2,  { 0.0, 0.0, -0.08 },
      { 0.005,0,0, 0,0.005,0, 0,0,0.002 } },
};

/* fixed rather than rand(): a failure has to be reproducible */
static const double QS[3][3]   = { {  0.3, -0.7,  0.12 }, {  1.1, 0.4, -0.05 }, { -0.9, 1.3,  0.20 } };
static const double QDS[3][3]  = { {  0.5, -0.2,  0.30 }, { -1.0, 0.7,  0.10 }, {  0.2, 0.9, -0.40 } };
static const double QDDS[3][3] = { {  1.5, -0.6,  0.20 }, {  0.3, 2.0, -0.70 }, { -1.1, 0.4,  0.80 } };

/* ---- 4. the qdd part of inverse dynamics is exactly M(q) qdd ------------ */

static void test_inertia_consistency(void)
{
    dyn_chain c = { spatial3, 3, { 0.0, 0.0, -G } };
    int t, i, j;

    for (t = 0; t < 3; t++) {
        double with[3], without[3], M[9];

        assert(dyn_inverse(&c, QS[t], QDS[t], QDDS[t], with) == DYN_OK);
        assert(dyn_inverse(&c, QS[t], QDS[t], NULL, without) == DYN_OK);
        assert(dyn_mass_matrix(&c, QS[t], M) == DYN_OK);

        for (i = 0; i < 3; i++) {
            double s = 0.0;
            for (j = 0; j < 3; j++) s += M[i * 3 + j] * QDDS[t][j];
            assert(NEAR(with[i] - without[i], s, 1e-9));
        }
    }
}

/* ---- 5. forward dynamics undoes inverse dynamics ------------------------ */

static void test_round_trip(void)
{
    dyn_chain c = { spatial3, 3, { 0.0, 0.0, -G } };
    int t, i;

    for (t = 0; t < 3; t++) {
        double tau[3], qdd[3];
        assert(dyn_inverse(&c, QS[t], QDS[t], QDDS[t], tau) == DYN_OK);
        assert(dyn_forward(&c, QS[t], QDS[t], tau, qdd) == DYN_OK);
        for (i = 0; i < 3; i++) assert(NEAR(qdd[i], QDDS[t][i], 1e-6));
    }
}

/* ---- 6. a prismatic joint lifting its own mass -------------------------- */

static void test_prismatic_lift(void)
{
    static const dyn_link slider[1] = {
        { 0.0, 0.0, 0.0, 0.0, DYN_PRISMATIC, 3.4, { 0.0, 0.0, 0.0 }, NO_INERTIA },
    };
    static const double accels[3] = { 0.0, 1.5, -2.0 };
    dyn_chain c = { slider, 1, { 0.0, 0.0, -G } };
    double q = 0.2, qd = 0.0;
    int i;

    for (i = 0; i < 3; i++) {
        double f;
        assert(dyn_inverse(&c, &q, &qd, &accels[i], &f) == DYN_OK);
        assert(NEAR(f, 3.4 * (accels[i] + G), 1e-9));   /* newtons, not newton-metres */
    }
}

/* ---- 7. bad arguments ---------------------------------------------------- */

static void test_bad_args(void)
{
    static const dyn_link negative_mass[1] = {
        { 0.0, 0.3, 0.0, 0.0, DYN_REVOLUTE, -1.0, { 0.0, 0.0, 0.0 }, NO_INERTIA },
    };
    dyn_chain good   = { planar2, 2, { 0.0, -G, 0.0 } };
    dyn_chain none   = { planar2, 0, { 0.0, -G, 0.0 } };
    dyn_chain toomany= { planar2, DYN_MAX_JOINTS + 1, { 0.0, -G, 0.0 } };
    dyn_chain nolinks= { NULL,    2, { 0.0, -G, 0.0 } };
    dyn_chain heavy  = { negative_mass, 1, { 0.0, -G, 0.0 } };
    double q[2] = { 0.1, 0.2 }, notq[2] = { 0.1, 0.0 };
    double tau[2] = { 0.0, 0.0 }, qdd[2], M[4];

    notq[1] = NAN;

    assert(dyn_gravity(NULL, q, tau)     == DYN_EARG);
    assert(dyn_gravity(&good, q, NULL)   == DYN_EARG);
    assert(dyn_gravity(&good, NULL, tau) == DYN_EARG);
    assert(dyn_gravity(&none, q, tau)    == DYN_EARG);
    assert(dyn_gravity(&toomany, q, tau) == DYN_EARG);
    assert(dyn_gravity(&nolinks, q, tau) == DYN_EARG);
    assert(dyn_gravity(&heavy, q, tau)   == DYN_EARG);
    assert(dyn_gravity(&good, notq, tau) == DYN_EARG);

    assert(dyn_inverse(&good, q, NULL, NULL, NULL) == DYN_EARG);
    assert(dyn_inverse(&good, q, notq, NULL, tau)  == DYN_EARG);
    assert(dyn_mass_matrix(&good, q, NULL)         == DYN_EARG);
    assert(dyn_mass_matrix(&none, q, M)            == DYN_EARG);
    assert(dyn_forward(&good, q, NULL, tau, NULL)  == DYN_EARG);
    assert(dyn_forward(&good, q, NULL, NULL, qdd)  == DYN_EARG);
    assert(dyn_forward(&heavy, q, NULL, tau, qdd)  == DYN_EARG);
}

int main(void)
{
    test_point_mass_pendulum();
    test_planar_gravity();
    test_planar_mass_matrix();
    test_inertia_consistency();
    test_round_trip();
    test_prismatic_lift();
    test_bad_args();
    printf("all tests passed\n");
    return 0;
}
