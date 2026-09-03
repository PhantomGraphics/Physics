# Phantom::Physics Headless SPH Examples

These examples demonstrate SPH fluid simulation without a window or GPU. They
link only `PhysicsCore` and are the easiest entry point to the module.

| Example | Description | Main APIs |
|---|---|---|
| `01_dam_break.cpp` | WCSPH dam break exported as PLY | `WCSPHFluid`, `WCSPHSolver` |
| `02_faucet.cpp` | Faucet with emitter and outflow | Emitter and outflow APIs |
| `03_solver_interface.cpp` | One scene with WCSPH and PBSPH | `ISPHSolver` |

## Build and run

Use C++17, CMake 3.20+, and Ninja. Vulkan is not required.

```powershell
cmake -S Physics -B Physics/build_examples -DCMAKE_BUILD_TYPE=Release
cmake --build Physics/build_examples

./Physics/build_examples/examples/sph_example_01_dam_break.exe [output-directory]
./Physics/build_examples/examples/sph_example_02_faucet.exe [output-directory]
./Physics/build_examples/examples/sph_example_03_solver_interface.exe
```

Alternatively, configure `Physics/examples` directly. Use
`-DPHYSICS_BUILD_EXAMPLES=OFF` to omit examples or `--target` to build one.
Release builds are strongly recommended.

The first two programs write binary little-endian PLY position sequences,
which can be opened in Blender, MeshLab, CloudCompare, Houdini, or Open3D.

## Programming model

Fluids own particle data and parameters; solvers retain non-owning pointers,
so every fluid must outlive its solver.

```cpp
WCSPHFluid fluid;
WCSPHSolver solver;
solver.add(&fluid);
fluid.updateEmitters(dt);
solver.simulate(dt, maxIterations);
fluid.removeOutflowParticles();
```

`WCSPHSolver`, `DFSPHSolver`, and `PBSPHSolver` implement `ISPHSolver`.
Start with WCSPH. PBSPH is stable with tuned small-scale stiffness; DFSPH is
more sensitive to adaptive-substep parameters. Inspect `PhysicsTest/` and
`PhysicsView/scenarios/` for further validated examples.
