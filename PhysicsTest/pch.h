//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

// Tests that take several seconds even in Release (and ~10x that in Debug) are
// skipped in unoptimized builds; run them from a Release build. Prefer moving
// long simulations to PhysicsView scenarios (scenarios/*.json, "debug-slow" tag).
#ifdef NDEBUG
#define SKIP_IN_DEBUG_SLOW() ((void)0)
#else
#define SKIP_IN_DEBUG_SLOW() GTEST_SKIP() << "slow in Debug; run in Release"
#endif
