/* A differential-drive rover walking a square: drive a side, turn 90 degrees, repeat.
 * It prints the wheel speeds it would command and where dead reckoning thinks it is. */
#include "mob.h"
#include <stdio.h>

#define PI 3.14159265358979323846
#define DT 0.01   /* a 100 Hz control loop */

/*                            type       radius  track  wheelbase */
static const mob_base rover = { MOB_DIFF,  0.05,  0.40,  0.00 };

/* Hold one body velocity for `secs`, integrating the pose as we go. */
static void drive(mob_pose *p, mob_twist v, double secs)
{
    double w[MOB_MAX_WHEELS];
    int i, steps = (int)(secs / DT + 0.5);

    if (mob_inverse(&rover, &v, w) != MOB_OK) {
        printf("  cannot do that\n");
        return;
    }
    printf("  vx %5.2f m/s  wz %5.2f rad/s  ->  left %6.2f   right %6.2f rad/s\n",
           v.vx, v.wz, w[0], w[1]);

    for (i = 0; i < steps; i++) mob_integrate(p, &v, DT);
}

static void show(const mob_pose *p)
{
    printf("  pose  x %6.3f  y %6.3f  theta %6.3f rad\n\n", p->x, p->y, p->theta);
}

int main(void)
{
    mob_pose p = { 0.0, 0.0, 0.0 };
    mob_twist go   = { 0.5, 0.0, 0.0 };      /* 0.5 m/s for 2 s = a 1 m side  */
    mob_twist turn = { 0.0, 0.0, PI / 4 };   /* 45 deg/s for 2 s = a 90 deg turn */
    int side;

    printf("differential rover, 10 cm wheels 40 cm apart, walking a 1 m square\n\n");

    for (side = 1; side <= 4; side++) {
        printf("side %d: drive 2 s\n", side);
        drive(&p, go, 2.0);
        show(&p);

        printf("side %d: turn 90 deg on the spot\n", side);
        drive(&p, turn, 2.0);
        show(&p);
    }

    printf("four sides later it is back where it started: x %.3f  y %.3f\n", p.x, p.y);
    printf("on a real robot it would not be -- see \"odometry drifts\" in the README.\n");
    return 0;
}
