/* Self-check: assert-based, no framework. Build with `make check`. */
#include "mob.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
#define NEAR(a, b) (fabs((a) - (b)) < 1e-9)

/*                          type            radius  track  wheelbase */
static const mob_base diff = { MOB_DIFF,      0.05,   0.40,  0.00 };
static const mob_base mec  = { MOB_MECANUM,   0.05,   0.40,  0.30 };
static const mob_base car  = { MOB_ACKERMANN, 0.30,   0.00,  2.50 };

static void test_diff_numbers(void)
{
    mob_twist v = { 1.0, 0.0, 0.0 };
    double w[MOB_MAX_WHEELS];

    assert(mob_wheels(&diff) == 2);

    /* 1 m/s on a 0.05 m wheel is 20 rad/s, both sides the same. */
    assert(mob_inverse(&diff, &v, w) == MOB_OK);
    assert(NEAR(w[0], 20.0) && NEAR(w[1], 20.0));

    /* Spin on the spot: each wheel walks a 0.2 m arm, opposite ways. */
    v.vx = 0.0; v.wz = 1.0;
    assert(mob_inverse(&diff, &v, w) == MOB_OK);
    assert(NEAR(w[0], -4.0) && NEAR(w[1], 4.0));
}

static void roundtrip(const mob_base *b, mob_twist in)
{
    double w[MOB_MAX_WHEELS];
    mob_twist out;

    assert(mob_inverse(b, &in, w) == MOB_OK);
    assert(mob_forward(b, w, &out) == MOB_OK);
    assert(NEAR(out.vx, in.vx) && NEAR(out.vy, in.vy) && NEAR(out.wz, in.wz));
}

static void test_roundtrips(void)
{
    mob_twist straight_ish = { 0.70,  0.00, -0.30 };
    mob_twist crabbing     = { 0.70, -0.40,  0.90 };
    mob_twist cornering    = { 1.20,  0.00,  0.25 };

    roundtrip(&diff, straight_ish);   /* vy must be 0: it cannot slide sideways */
    roundtrip(&mec,  crabbing);       /* mecanum is the one that can */
    roundtrip(&car,  cornering);      /* nor can a car */
}

static void test_mecanum_sideways(void)
{
    mob_twist v = { 0.0, 1.0, 0.0 };     /* pure strafe to the left */
    double w[MOB_MAX_WHEELS];

    assert(mob_wheels(&mec) == 4);
    assert(mob_inverse(&mec, &v, w) == MOB_OK);

    /* Diagonal pairs match; the two diagonals turn opposite ways. */
    assert(NEAR(w[0], w[3]));            /* front-left  == rear-right */
    assert(NEAR(w[1], w[2]));            /* front-right == rear-left  */
    assert(NEAR(w[0], -w[1]));
    assert(w[0] < 0.0 && w[1] > 0.0);
}

static void test_ackermann_cannot_spin(void)
{
    mob_twist spin = { 0.0, 0.0, 1.0 };
    mob_twist stop = { 0.0, 0.0, 0.0 };
    double w[MOB_MAX_WHEELS];

    assert(mob_wheels(&car) == 2);
    assert(mob_inverse(&car, &spin, w) == MOB_EARG);   /* no car turns on the spot */

    assert(mob_inverse(&car, &stop, w) == MOB_OK);     /* but standing still is fine */
    assert(NEAR(w[0], 0.0) && NEAR(w[1], 0.0));
}

static void test_ackermann_reverse(void)
{
    mob_twist backing = { -1.0, 0.0, 1.0 };   /* reversing while turning */
    double w[MOB_MAX_WHEELS];

    assert(mob_inverse(&car, &backing, w) == MOB_OK);
    /* Front wheels are on a hinge: the steer angle has to stay inside +-90
     * degrees. atan keeps it there, atan2 would report 112 degrees instead. */
    assert(fabs(w[1]) < PI / 2);
    assert(w[0] < 0.0);                       /* rear axle running backwards */
}

static void test_integrate_circle(void)
{
    mob_pose p = { 0.0, 0.0, 0.0 };
    mob_twist v = { 1.0, 0.0, 1.0 };     /* 1 m/s and 1 rad/s: a circle of radius 1 */
    double dt = 2.0 * PI / 1000.0;
    int i;

    for (i = 0; i < 1000; i++) mob_integrate(&p, &v, dt);

    assert(fabs(p.x) < 1e-6 && fabs(p.y) < 1e-6);      /* one lap, back to the start */
    assert(fabs(p.theta) < 1e-6);                      /* 2*pi wraps to 0 */
}

static void test_integrate_arc(void)
{
    mob_pose p = { 0.0, 0.0, 0.0 };
    mob_twist v = { 1.0, 0.0, 1.0 };     /* radius 1, turning left */

    /* A quarter turn in a single coarse step. The arc is exact, so this lands
     * on (1, 1); a straight-line step would put it out at (pi/2, 0). */
    mob_integrate(&p, &v, PI / 2);
    assert(fabs(p.x - 1.0) < 1e-9 && fabs(p.y - 1.0) < 1e-9);
    assert(fabs(p.theta - PI / 2) < 1e-9);
}

static void test_integrate_straight(void)
{
    mob_pose p = { 0.0, 0.0, PI / 2 };   /* pointing along +y */
    mob_twist v = { 1.0, 0.0, 0.0 };

    mob_integrate(&p, &v, 1.0);
    assert(fabs(p.x) < 1e-6 && fabs(p.y - 1.0) < 1e-6);
    assert(fabs(p.theta - PI / 2) < 1e-6);
}

static void test_bad_args(void)
{
    mob_base bad = diff;
    mob_twist v = { 1.0, 0.0, 0.0 };
    mob_twist out;
    double w[MOB_MAX_WHEELS] = { 0.0, 0.0, 0.0, 0.0 };

    bad.wheel_radius = 0.0;
    assert(mob_inverse(&bad, &v, w) == MOB_EARG);
    assert(mob_forward(&bad, w, &out) == MOB_EARG);

    bad = diff; bad.track = 0.0;         /* a base needs the dimensions it uses */
    assert(mob_inverse(&bad, &v, w) == MOB_EARG);

    bad = mec; bad.wheelbase = 0.0;
    assert(mob_inverse(&bad, &v, w) == MOB_EARG);

    assert(mob_wheels(NULL) == MOB_EARG);
    assert(mob_inverse(NULL, &v, w) == MOB_EARG);
    assert(mob_inverse(&diff, NULL, w) == MOB_EARG);
    assert(mob_inverse(&diff, &v, NULL) == MOB_EARG);
    assert(mob_forward(&diff, NULL, &out) == MOB_EARG);
    assert(mob_forward(&diff, w, NULL) == MOB_EARG);

    v.vx = NAN;                          /* garbage in, error out */
    assert(mob_inverse(&diff, &v, w) == MOB_EARG);
    w[0] = NAN;
    assert(mob_forward(&diff, w, &out) == MOB_EARG);

    /* mob_integrate has no return code, so it must simply not move the pose. */
    {
        mob_pose p = { 1.0, 2.0, 0.5 };
        mob_twist nan_v = { NAN, 0.0, 0.0 };
        mob_twist ok_v = { 1.0, 0.0, 0.0 };
        mob_integrate(&p, &nan_v, 1.0);
        mob_integrate(&p, &ok_v, NAN);
        assert(NEAR(p.x, 1.0) && NEAR(p.y, 2.0) && NEAR(p.theta, 0.5));
    }
}

int main(void)
{
    test_diff_numbers();
    test_roundtrips();
    test_mecanum_sideways();
    test_ackermann_cannot_spin();
    test_ackermann_reverse();
    test_integrate_circle();
    test_integrate_arc();
    test_integrate_straight();
    test_bad_args();
    printf("all tests passed\n");
    return 0;
}
