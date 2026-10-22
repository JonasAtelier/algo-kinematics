/* Gravity compensation for a small 2-link planar arm: the torque each motor
 * has to hold at a pose just to stop the arm falling. Then one line of
 * forward dynamics -- switch the motors off and see what it does. */
#include "dyn.h"
#include <stdio.h>
#include <math.h>

#define PI  3.14159265358979323846
#define DEG (PI / 180.0)

#define L1 0.30
#define L2 0.25
#define M1 1.5
#define M2 0.8

/* Uniform rods: centre of mass at mid-link, inertia m l^2 / 12 about the two
 * axes across the rod. The link frame's origin is at the far end of the link,
 * so mid-link is half a link back along -x. */
static const dyn_link arm2[2] = {
    /*  d    a   alpha theta  type          mass  com                     inertia */
    {  0.0, L1,  0.0,  0.0,   DYN_REVOLUTE, M1,   { -L1 / 2, 0.0, 0.0 },
       { 0,0,0,  0, M1*L1*L1/12.0, 0,  0,0, M1*L1*L1/12.0 } },
    {  0.0, L2,  0.0,  0.0,   DYN_REVOLUTE, M2,   { -L2 / 2, 0.0, 0.0 },
       { 0,0,0,  0, M2*L2*L2/12.0, 0,  0,0, M2*L2*L2/12.0 } },
};

int main(void)
{
    static const double shoulder[3] = { 0.0, 45.0, 90.0 };
    static const double elbow[3]    = { 0.0, -45.0, -90.0 };
    dyn_chain arm = { arm2, 2, { 0.0, -9.81, 0.0 } };
    double q[2], tau[2], qdd[2];
    int i, j;

    printf("2-link planar arm, upright plane, gravity 9.81 m/s^2 down the -y axis.\n");
    printf("link 1: %.2f m, %.2f kg     link 2: %.2f m, %.2f kg     (uniform rods)\n\n",
           L1, M1, L2, M2);
    printf("  shoulder     elbow     hold shoulder      hold elbow\n");

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++) {
            q[0] = shoulder[i] * DEG;
            q[1] = elbow[j] * DEG;
            if (dyn_gravity(&arm, q, tau) != DYN_OK) return 1;
            printf("  %5.0f deg  %5.0f deg    %8.3f N m      %8.3f N m\n",
                   shoulder[i], elbow[j], tau[0], tau[1]);
        }

    /* Same arm, straight out, motors switched off. */
    q[0] = 0.0; q[1] = 0.0;
    tau[0] = 0.0; tau[1] = 0.0;
    if (dyn_forward(&arm, q, NULL, tau, qdd) != DYN_OK) return 1;

    printf("\nLet go at shoulder 0 deg, elbow 0 deg with no torque at all:\n");
    printf("  the shoulder accelerates at %.2f rad/s^2, the elbow at %.2f rad/s^2\n",
           qdd[0], qdd[1]);
    return 0;
}
