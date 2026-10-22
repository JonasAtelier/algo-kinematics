/* Self-check: assert-based, no framework. Build with `make check`. */
#include "kin.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
#define NEAR(a, b) (fabs((a) - (b)) < 1e-6)

static const kin_link planar2[2] = {
    { 0.0, 1.0, 0.0, 0.0, KIN_REVOLUTE, 0, 0 },
    { 0.0, 1.0, 0.0, 0.0, KIN_REVOLUTE, 0, 0 },
};

static const kin_link slider[1] = {
    { 0.0, 0.0, 0.0, 0.0, KIN_PRISMATIC, 0, 0 },
};

static const kin_link ur5[6] = {
    { 0.089159,  0.0,      PI / 2,  0.0, KIN_REVOLUTE, -2*PI, 2*PI },
    { 0.0,      -0.425,    0.0,     0.0, KIN_REVOLUTE, -2*PI, 2*PI },
    { 0.0,      -0.39225,  0.0,     0.0, KIN_REVOLUTE, -PI,   PI   },
    { 0.10915,   0.0,      PI / 2,  0.0, KIN_REVOLUTE, -2*PI, 2*PI },
    { 0.09465,   0.0,     -PI / 2,  0.0, KIN_REVOLUTE, -2*PI, 2*PI },
    { 0.0823,    0.0,      0.0,     0.0, KIN_REVOLUTE, -2*PI, 2*PI },
};

static void test_fk_planar(void)
{
    kin_chain c = { planar2, 2 };
    kin_mat4 t;
    double q[2];

    q[0] = 0.0; q[1] = 0.0;
    assert(kin_forward(&c, q, t) == KIN_OK);
    assert(NEAR(t[3], 2.0) && NEAR(t[7], 0.0));

    q[0] = PI / 2; q[1] = 0.0;
    kin_forward(&c, q, t);
    assert(NEAR(t[3], 0.0) && NEAR(t[7], 2.0));

    q[0] = 0.0; q[1] = PI / 2;
    kin_forward(&c, q, t);
    assert(NEAR(t[3], 1.0) && NEAR(t[7], 1.0));
}

static void test_fk_prismatic(void)
{
    kin_chain c = { slider, 1 };
    kin_mat4 t;
    double q = 0.37;
    kin_forward(&c, &q, t);
    assert(NEAR(t[11], 0.37));
}

static void test_jacobian_vs_numeric(void)
{
    kin_chain c = { ur5, 6 };
    double q[6] = { 0.2, -0.9, 0.7, 0.4, -1.1, 0.5 };
    double J[36];
    int j;

    assert(kin_jacobian(&c, q, J) == KIN_OK);

    for (j = 0; j < 6; j++) {          /* linear rows against finite difference */
        kin_mat4 a, b;
        double h = 1e-6, qq[6];
        int i;
        for (i = 0; i < 6; i++) qq[i] = q[i];
        qq[j] = q[j] - h; kin_forward(&c, qq, a);
        qq[j] = q[j] + h; kin_forward(&c, qq, b);
        assert(fabs((b[3]  - a[3])  / (2 * h) - J[0 * 6 + j]) < 1e-5);
        assert(fabs((b[7]  - a[7])  / (2 * h) - J[1 * 6 + j]) < 1e-5);
        assert(fabs((b[11] - a[11]) / (2 * h) - J[2 * 6 + j]) < 1e-5);
    }
}

static void test_ik_roundtrip(void)
{
    kin_chain c = { ur5, 6 };
    double q[6]    = { 0.3, -1.2, 1.0, -0.6, 1.4, 0.2 };
    double seed[6] = { 0.0, -1.0, 0.8, -0.4, 1.0, 0.0 };
    double sol[6], a[6], b[6];
    kin_mat4 target, reached;
    kin_inverse_opts o = { 0 };
    int i;

    o.max_iters = 300;
    kin_forward(&c, q, target);
    assert(kin_inverse(&c, target, seed, sol, &o) == KIN_OK);

    kin_forward(&c, sol, reached);
    kin_pose_decompose(target, a);
    kin_pose_decompose(reached, b);
    for (i = 0; i < 6; i++) assert(fabs(a[i] - b[i]) < 1e-3);
}

static void test_ik_bad_args(void)
{
    kin_chain c = { ur5, 6 };
    kin_mat4 t;
    double q[6] = { 0 };
    assert(kin_forward(NULL, q, t) == KIN_EARG);
    assert(kin_inverse(&c, t, q, NULL, NULL) == KIN_EARG);
}

static void test_pose_roundtrip(void)
{
    kin_mat4 t;
    double p[6];
    kin_pose(0.1, -0.2, 0.3, 0.4, -0.5, 0.6, t);
    kin_pose_decompose(t, p);
    assert(NEAR(p[0], 0.1) && NEAR(p[1], -0.2) && NEAR(p[2], 0.3));
    assert(NEAR(p[3], 0.4) && NEAR(p[4], -0.5) && NEAR(p[5], 0.6));
}

int main(void)
{
    test_fk_planar();
    test_fk_prismatic();
    test_jacobian_vs_numeric();
    test_ik_roundtrip();
    test_ik_bad_args();
    test_pose_roundtrip();
    printf("all tests passed\n");
    return 0;
}
