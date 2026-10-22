/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "dyn.h"

#include <cassert>
#include <cmath>

int main()
{
	static const dyn_link links[2] = {
		{ 0.0, 1.0, 0.0, 0.0, DYN_REVOLUTE, 2.0, { 0.0, 0.0, 0.0 }, { 0,0,0, 0,0,0, 0,0,0 } },
		{ 0.0, 1.0, 0.0, 0.0, DYN_REVOLUTE, 1.0, { 0.0, 0.0, 0.0 }, { 0,0,0, 0,0,0, 0,0,0 } },
	};
	dyn_chain arm = { links, 2, { 0.0, -9.81, 0.0 } };
	double q[2] = { 0.0, 0.0 };
	double tau[2], qdd[2];

	assert(dyn_gravity(&arm, q, tau) == DYN_OK);
	/* straight out along x: shoulder holds both masses, elbow holds one */
	assert(std::fabs(tau[0] - (3.0 * 9.81 * 1.0 + 1.0 * 9.81 * 1.0)) < 1e-9);

	assert(dyn_forward(&arm, q, NULL, tau, qdd) == DYN_OK);
	assert(std::fabs(qdd[0]) < 1e-9);	/* held exactly: it does not move */
	return 0;
}
