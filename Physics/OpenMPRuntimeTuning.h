#pragma once

namespace Phantom {
	namespace Physics {

		// Sets OMP_WAIT_POLICY=PASSIVE for this process (once, and only if
		// nothing -- caller or environment -- has already set it explicitly)
		// before the process's first OpenMP parallel region runs. Idle worker
		// threads otherwise keep spinning after every parallel region (MSVC
		// OpenMP's default ACTIVE wait policy), stealing CPU time from the
		// serial phases that alternate with parallel ones in
		// WCSPHSolver/PBSPHSolver/DFSPHSolver::simulate() -- measured to make
		// those serial phases grow *with* thread count instead of staying
		// flat, which is why throughput peaks around 8 threads and regresses
		// above it (docs/issue/wcsph_parallel_scaling_profile.md section 5).
		// Must run before the first #pragma omp parallel region the calling
		// process executes -- every simulate() below calls it as its first
		// statement for exactly that reason.
		void ensurePassiveOpenMPWaitPolicy();

	}
}
