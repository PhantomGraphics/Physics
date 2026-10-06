# PBVR smoke self-shadow

PBVR smoke retains Poisson particle generation, opaque sub-particle depth
testing and ensemble averaging. Self-shadow uses the existing uniform-sphere
smoke puffs directly; there is no density grid or ray-marched rendering pass.

`FlameSmokeShadow` builds a BVH over the puffs and integrates the intersection
lengths of a ray from each puff centre toward the shared directional light.
The optical depth per unit length is `smokeExtinction * density *
pbvrDensityScale / diameter`, consistent with the generator's uncapped PBVR
extinction model. The receiving puff is excluded from the CPU calculation.
The GPU adds its own puff's optical depth at each generated sub-particle, then
applies `exp(-smokeShadowStrength * opticalDepth)` to directional illumination.
Thermal emission is unaffected. No additional random numbers are consumed.

External shadow is a centre-sampled approximation: neighbouring puffs' shadow
boundaries inside a single receiving puff are not resolved. Its own shadow is
resolved per sub-particle. Particle generation caps can also make PBVR view
extinction differ from the uncapped lighting model in extreme configurations.
Solid-object shadows and flame-as-light-source illumination are outside this
pass's scope.

The light comes from PhysicsView's existing `SetLight` command / Rendering
panel. Its direction specifies light travel, so shadow rays use the opposite
direction. Colour and intensity affect scattered illumination. Ambient fill
stays unoccluded. Normal mode retains its previous colour calculation.

Flame panel controls (PBVR mode):

- `Smoke Self Shadow`: strength, default 1; 0 restores unshadowed PBVR colour.
- `Smoke Shadow Fill`: unoccluded fill fraction, default 0.25, range 0..1.

CLI equivalents:

```text
SetFlameRenderParam:smokeShadowStrength,1
SetFlameRenderParam:smokeShadowAmbient,0.25
```

Geometry, optical density and light-direction changes invalidate the cached
CPU shadow depths. Shading and light changes reset the PBVR image history.
A paused, converged scene therefore does not repeatedly rebuild the BVH.
Dense overlapping puffs can still make ray tracing expensive in moving scenes.

Verification:

```powershell
.\build\windows-debug\Physics\PhysicsTest.exe --gtest_filter=FlameSmokeShadow.*
.\Physics\PhysicsView\run_physics_scenarios.ps1 -Configuration Debug -Filter '48_flame_*'
```

`48_flame_self_shadow.json` captures shadow off, on, and reversed light at the
same paused state. It disables thermal smoke glow and uses unit white light
to isolate the self-shadow effect, and checks history reset, convergence,
parameter bounds and particle-buffer overflow.

## Isolated white particles

The burner previously developed high-frequency scalar oscillations in sparse
particle pairs: the `kappa * dt / h^2` cap did not bound the actual SPH diffusion
row sums. Temperatures could alternate below zero and above the 2600 K ceiling,
creating isolated white emitters high in the plume. `FlameSolver` now limits the
discrete outgoing weight to 1/2 with a common scale per scalar (temperature,
fuel, oxygen and soot). This preserves pairwise conservation and prevents
diffusion from introducing new extrema. As with the existing diffusivity cap,
unstable configurations receive reduced effective diffusion rather than a
larger step. `SparsePairDiffusionPreservesExtremaAndTotals` reproduces the old
failure and checks bounds and totals over repeated steps.
