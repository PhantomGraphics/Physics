# Phantom Physics

Phantom Physics is a C++17 physics simulation module. It provides SPH-based fluid,
rigid-body, and XPBD soft-body solvers, coupled rigid/fluid/soft-body simulation,
and standalone Vulkan viewers.

> This repository is the `Physics/` submodule of the
> [Phantom](https://github.com/PhantomGraphics/Phantom) superproject. It can be
> configured separately, but its build expects the sibling `CGLib/` directory
> and shared `cmake/` modules from the superproject.

## Projects

| Project | Type | Purpose |
|---|---|---|
| `Physics/` (`PhysicsCore`) | Static library | Core library in `Phantom::Physics` |
| `PhysicsTest/` | GoogleTest | Core-library unit tests |
| `PhysicsView/` | Vulkan + ImGui app | Integrated viewer with JSON scenarios |
| `Fluid_GPU_Vk/` (`FluidGPUVkCore`) | Static library | Vulkan Compute CSPH solver |
| `FluidRenderer/` (`FluidRendererCore`) | Static library | Screen-space fluid rendering pipeline |
| `FlameView/` | Vulkan + ImGui app | Experimental combustion-gas SPH viewer |

### Core capabilities

- **Fluids (SPH):** the common `ISPHSolver` interface and `DFSPHSolver`,
  `PBSPHSolver`, `WCSPHSolver`, and `FlameSolver`; one-way SDF penalty and
  two-way Akinci boundary-particle coupling; emitters, outflow regions, and
  analytic plane, sphere, and plate boundaries.
- **Rigid bodies:** `RigidBody` and the broad-phase BVH, narrow-phase, contact
  manifold, and `RigidBodySolver` collision pipeline.
- **Soft bodies (XPBD):** cloth, jelly, and rope bodies, constraints, and the
  `XPBDSolver` / `SoftBodySolver` pair.
- **Three-way coupling:** the top-level `PhysicsSolver`, which coordinates
  `RigidFluidSolver`, `SoftFluidSolver`, and `RigidSoftSolver`.

## Build

CMake is the supported build system. The project requires C++17 and Ninja;
viewer targets require Vulkan SDK 1.4.341.1 or later.

```powershell
# Configure and build Physics from the Phantom repository root.
cmake -S Physics -B Physics/build_windows -DCMAKE_BUILD_TYPE=Debug
cmake --build Physics/build_windows

# Or build the complete repository through a root preset.
cmake --preset windows-debug
cmake --build --preset windows-debug
```

Main targets are `PhysicsCore`, `PhysicsTest`, `PhysicsView`, `FlameView`,
`Fluid_GPU_Vk` (`FluidGPUVkCore`), and `FluidRenderer` (`FluidRendererCore`).

If Vulkan headers, the Vulkan loader, or GLFW cannot be found, CMake skips
`PhysicsView` and `FlameView` with a warning while retaining the core and test
targets. On Linux, provide `VULKAN_INCLUDE_DIR`, `VULKAN_LIBRARY`, and
`GLFW_LIBRARY` to enable the viewers.

Precompiled SPIR-V shaders are included. Run a project's
`shaders/compile_shaders.bat` only after changing GLSL sources.

## Tests

```powershell
./build/windows-debug/Physics/PhysicsTest.exe
./build/windows-debug/Physics/PhysicsTest.exe --gtest_filter=DFSPHSolverTest.*
ctest --preset windows-debug -R PhysicsTest
```

### PhysicsView scenario tests

`PhysicsView` supports automated JSON scenarios named
`NN_<domain>_<name>.json`; the numeric prefix also defines execution order.

```powershell
./Physics/PhysicsView/run_physics_scenarios.ps1 -Configuration Debug
./Physics/PhysicsView/run_physics_scenarios.ps1 -List
./Physics/PhysicsView/run_physics_scenarios.ps1 -Filter '1*_fluid_*'
./build/windows-debug/Physics/PhysicsView.exe --run-scenario Physics/PhysicsView/scenarios/10_fluid_dfsph_pool_settle.json
```

## Technology stack

C++17, Vulkan, VMA, GLFW, Dear ImGui, GLM 0.9.9.8, Eigen 3.4.0, OpenMP,
and GoogleTest.

## License

[MIT License](LICENSE) — Copyright (c) 2026 PhantomGraphics. Bundled and
external dependencies remain subject to their respective licenses.
