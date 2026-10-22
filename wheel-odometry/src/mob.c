#include "mob.h"
#include <math.h>

#define PI 3.14159265358979323846

/* ---- validation ------------------------------------------------------ */

/* The radius always matters. The two dimensions do not, so check only the
 * ones this base type actually uses -- an Ackermann car has no track here,
 * and demanding one would reject a perfectly good base. */
static int base_ok(const mob_base *b)
{
    if (!b || !isfinite(b->wheel_radius) || b->wheel_radius <= 0.0) return 0;

    switch (b->type) {
    case MOB_DIFF:
        return isfinite(b->track) && b->track > 0.0;
    case MOB_MECANUM:
        return isfinite(b->track) && b->track > 0.0 &&
               isfinite(b->wheelbase) && b->wheelbase > 0.0;
    case MOB_ACKERMANN:
        return isfinite(b->wheelbase) && b->wheelbase > 0.0;
    }
    return 0;
}

static int twist_ok(const mob_twist *v)
{
    return v && isfinite(v->vx) && isfinite(v->vy) && isfinite(v->wz);
}

/* Half the diagonal of the mecanum wheel rectangle: the lever arm each wheel
 * has about the centre of the base. */
static double mec_arm(const mob_base *b)
{
    return b->wheelbase / 2.0 + b->track / 2.0;
}

/* ---- public ---------------------------------------------------------- */

int mob_wheels(const mob_base *b)
{
    if (!b) return MOB_EARG;

    switch (b->type) {
    case MOB_DIFF:      return 2;
    case MOB_MECANUM:   return 4;
    case MOB_ACKERMANN: return 2;   /* rear axle speed, then steer angle */
    }
    return MOB_EARG;
}

int mob_inverse(const mob_base *b, const mob_twist *v, double *wheels)
{
    double r, arm;

    if (!base_ok(b) || !twist_ok(v) || !wheels) return MOB_EARG;
    r = b->wheel_radius;

    switch (b->type) {
    case MOB_DIFF:
        /* The turn adds speed to the outer wheel and takes it off the inner. */
        wheels[0] = (v->vx - v->wz * b->track / 2.0) / r;
        wheels[1] = (v->vx + v->wz * b->track / 2.0) / r;
        return MOB_OK;

    case MOB_MECANUM:
        arm = mec_arm(b);
        wheels[0] = (v->vx - v->vy - v->wz * arm) / r;   /* front-left  */
        wheels[1] = (v->vx + v->vy + v->wz * arm) / r;   /* front-right */
        wheels[2] = (v->vx + v->vy - v->wz * arm) / r;   /* rear-left   */
        wheels[3] = (v->vx - v->vy + v->wz * arm) / r;   /* rear-right  */
        return MOB_OK;

    case MOB_ACKERMANN:
        /* A car cannot slide sideways, so a demanded vy is simply ignored. */
        if (v->vx == 0.0) {
            if (v->wz != 0.0) return MOB_EARG;   /* nor turn on the spot */
            wheels[0] = 0.0;                     /* stopped, wheels straight */
            wheels[1] = 0.0;
            return MOB_OK;
        }
        wheels[0] = v->vx / r;
        /* atan, not atan2: reversing must keep the steer inside +-90 degrees. */
        wheels[1] = atan(v->wz * b->wheelbase / v->vx);
        return MOB_OK;
    }
    return MOB_EARG;
}

int mob_forward(const mob_base *b, const double *wheels, mob_twist *v)
{
    double r, arm;
    int i, n;

    if (!base_ok(b) || !wheels || !v) return MOB_EARG;

    n = mob_wheels(b);
    for (i = 0; i < n; i++)
        if (!isfinite(wheels[i])) return MOB_EARG;   /* a dead encoder reads NaN */

    r = b->wheel_radius;

    switch (b->type) {
    case MOB_DIFF:
        v->vx = r * (wheels[1] + wheels[0]) / 2.0;
        v->vy = 0.0;
        v->wz = r * (wheels[1] - wheels[0]) / b->track;
        return MOB_OK;

    case MOB_MECANUM:
        /* Four wheels describe three velocities, so this is a least-squares
         * fit. For this layout it collapses to the usual signed averages. */
        arm = mec_arm(b);
        v->vx = r * ( wheels[0] + wheels[1] + wheels[2] + wheels[3]) / 4.0;
        v->vy = r * (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) / 4.0;
        v->wz = r * (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) / (4.0 * arm);
        return MOB_OK;

    case MOB_ACKERMANN:
        v->vx = wheels[0] * r;
        v->vy = 0.0;
        v->wz = v->vx * tan(wheels[1]) / b->wheelbase;
        return MOB_OK;
    }
    return MOB_EARG;
}

/* ---- dead reckoning -------------------------------------------------- */

/* Wrap to (-pi, pi]. fmod rather than a loop, so a big angle costs the same. */
static double wrap_pi(double a)
{
    double t = fmod(a + PI, 2.0 * PI);
    if (t <= 0.0) t += 2.0 * PI;
    return t - PI;
}

void mob_integrate(mob_pose *p, const mob_twist *v, double dt)
{
    double th, dth, s0, c0;

    if (!p || !twist_ok(v) || !isfinite(dt)) return;

    th  = p->theta;
    dth = v->wz * dt;
    s0 = sin(th);
    c0 = cos(th);

    if (fabs(dth) < 1e-9) {
        /* Straight enough over one step: rotate the body step into the world. */
        p->x += (v->vx * c0 - v->vy * s0) * dt;
        p->y += (v->vx * s0 + v->vy * c0) * dt;
    } else {
        /* Exact arc: the heading turns at a constant rate across the step, so
         * integrating the world velocity gives these closed forms. Much better
         * than a straight step when wz is large or dt is coarse. */
        double s1 = sin(th + dth), c1 = cos(th + dth);
        p->x += ( v->vx * (s1 - s0) + v->vy * (c1 - c0)) / v->wz;
        p->y += (-v->vx * (c1 - c0) + v->vy * (s1 - s0)) / v->wz;
    }
    p->theta = wrap_pi(th + dth);
}
