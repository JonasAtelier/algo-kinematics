/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "kin.h"

#include <cassert>
#include <cmath>

int main()
{
	static const kin_link links[2] = {
		{ 0.0, 1.0, 0.0, 0.0, KIN_REVOLUTE, 0.0, 0.0 },
		{ 0.0, 1.0, 0.0, 0.0, KIN_REVOLUTE, 0.0, 0.0 },
	};
	kin_chain arm = { links, 2 };
	double q[2] = { 0.0, 0.0 };
	double sol[2];
	kin_mat4 pose;

	assert(kin_forward(&arm, q, pose) == KIN_OK);
	assert(std::fabs(pose[3] - 2.0) < 1e-9);	/* both links straight out */

	assert(kin_inverse(&arm, pose, q, sol, NULL) == KIN_OK);
	return 0;
}
