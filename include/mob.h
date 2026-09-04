/* mob.h - wheel kinematics and dead-reckoning odometry for mobile bases.
 *
 * C99, no dependencies beyond libm, no malloc, no globals.
 * Link with -lm.
 */
#ifndef MOB_H
#define MOB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Return codes. */
#define MOB_OK    0
#define MOB_EARG (-1)   /* bad argument (NULL, not finite, impossible geometry) */

/* The most wheels[] entries any base uses. Size your arrays with this. */
#define MOB_MAX_WHEELS 4

typedef enum {
    MOB_DIFF,       /* two driven wheels on one axle: wheels[0] left, [1] right */
    MOB_MECANUM,    /* four mecanum wheels, rollers at 45 degrees in the usual X layout:
                     * [0] front-left, [1] front-right, [2] rear-left, [3] rear-right */
    MOB_ACKERMANN,  /* car: rear axle drives, front wheels steer. wheels[0] is rear axle
                     * speed (rad/s), wheels[1] is the steering angle (rad), bicycle model */
} mob_type;

typedef struct {
    mob_type type;
    double   wheel_radius;  /* m */
    double   track;         /* m, left wheel to right wheel (diff, mecanum) */
    double   wheelbase;     /* m, front axle to rear axle (mecanum, ackermann) */
} mob_base;

typedef struct { double vx, vy, wz; } mob_twist;   /* body frame: forward m/s, left m/s, rad/s counter-clockwise */
typedef struct { double x, y, theta; } mob_pose;   /* world frame, m, m, rad */

/* How many entries wheels[] uses for this base: 2, 4 or 2. MOB_EARG if b is NULL. */
int  mob_wheels(const mob_base *b);

/* Body velocity -> wheel commands. Ackermann: a demanded vy is ignored (a car cannot),
 * and the steer angle is atan(wz * wheelbase / vx); vx == 0 with wz != 0 is MOB_EARG,
 * while vx == 0 with wz == 0 is just "stopped" and gives two zeros. */
int  mob_inverse(const mob_base *b, const mob_twist *v, double *wheels);

/* Wheel measurements -> body velocity. Mecanum: least-squares of the four wheels,
 * which is the standard closed form. */
int  mob_forward(const mob_base *b, const double *wheels, mob_twist *v);

/* Dead reckoning: advance a world pose by one step of body velocity. Exact arc when
 * wz != 0, straight line otherwise. theta is wrapped to (-pi, pi]. No return code:
 * a NULL or non-finite argument leaves the pose untouched. */
void mob_integrate(mob_pose *p, const mob_twist *v, double dt);

#ifdef __cplusplus
}
#endif
#endif /* MOB_H */
