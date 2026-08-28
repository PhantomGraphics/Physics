#include "pch.h"

#include "OpenMPRuntimeTuning.h"

#include <cstdlib>

void Phantom::Physics::ensurePassiveOpenMPWaitPolicy()
{
	static const bool initialized = [] {
		if (std::getenv("OMP_WAIT_POLICY") != nullptr) {
			return true;
		}
#if defined(_WIN32)
		_putenv_s("OMP_WAIT_POLICY", "PASSIVE");
#else
		setenv("OMP_WAIT_POLICY", "PASSIVE", 0);
#endif
		return true;
	}();
	(void)initialized;
}
