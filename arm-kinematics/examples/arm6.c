/* 6-axis arm (UR5 DH parameters): forward kinematics, then inverse back to the same pose. */
#include "kin.h"
#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979323846

static const kin_link ur5[6] = {
    /*  d        a         alpha    theta  type            qmin    qmax  */
    { 0.089159,  0.0,      PI / 2,  0.0,   KIN_REVOLUTE,  -2*PI,  2*PI },
    { 0.0,      -0.425,    0.0,     0.0,   KIN_REVOLUTE,  -2*PI,  2*PI },
    { 0.0,      -0.39225,  0.0,     0.0,   KIN_REVOLUTE,  -PI,    PI   },
    { 0.10915,   0.0,      PI / 2,  0.0,   KIN_REVOLUTE,  -2*PI,  2*PI },
    { 0.09465,   0.0,     -PI / 2,  0.0,   KIN_REVOLUTE,  -2*PI,  2*PI },
    { 0.0823,    0.0,      0.0,     0.0,   KIN_REVOLUTE,  -2*PI,  2*PI },
};

static void print_pose(const char *tag, const kin_mat4 t)
{
    double p[6];
    kin_pose_decompose(t, p);
    printf("%-8s xyz = %7.4f %7.4f %7.4f   rpy = %7.4f %7.4f %7.4f\n",
           tag, p[0], p[1], p[2], p[3], p[4], p[5]);
}

int main(void)
{
    kin_chain arm = { ur5, 6 };
    double q[6]    = { 0.3, -1.2, 1.0, -0.6, 1.4, 0.2 };
    double seed[6] = { 0.0, -1.0, 0.8, -0.4, 1.0, 0.0 };
    double sol[6];
    kin_mat4 target, reached;
    int rc, i;

    kin_forward(&arm, q, target);
    print_pose("target", target);

    rc = kin_inverse(&arm, target, seed, sol, NULL);   /* NULL = default tuning */
    printf("inverse rc = %d\n", rc);

    printf("q solved:");
    for (i = 0; i < 6; i++) printf(" %7.4f", sol[i]);
    printf("\n");

    kin_forward(&arm, sol, reached);
    print_pose("reached", reached);

    return rc == KIN_OK ? 0 : 1;
}
