/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "mob.h"

#include <cassert>
#include <cmath>

int main()
{
	mob_base rover = { MOB_DIFF, 0.05, 0.4, 0.0 };
	mob_twist v = { 1.0, 0.0, 0.0 };
	mob_twist back;
	mob_pose p = { 0.0, 0.0, 0.0 };
	double w[MOB_MAX_WHEELS];

	assert(mob_wheels(&rover) == 2);

	assert(mob_inverse(&rover, &v, w) == MOB_OK);
	assert(std::fabs(w[0] - 20.0) < 1e-9);	/* 1 m/s on a 0.05 m wheel */

	assert(mob_forward(&rover, w, &back) == MOB_OK);
	assert(std::fabs(back.vx - 1.0) < 1e-9);

	mob_integrate(&p, &v, 1.0);
	assert(std::fabs(p.x - 1.0) < 1e-9);	/* one second straight ahead */
	return 0;
}
