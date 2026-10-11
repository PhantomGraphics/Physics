# PhysicsView: Vulkan resource ownership and lifetime

Scope: what `FluidApp` (and the pieces it drives) creates, who destroys it, and what has to be
re-done on swapchain recreation. The generic rules for the `VKG::` wrappers (explicit `destroy()`,
`create()` on a live object releases the old handles, nothing waits for the GPU) are in
`CGLib/docs/vulkan-ownership.md`; this page only records the PhysicsView-specific lifetimes.

Frame structure (`VkAppBase` main loop):
`onUpdate` (CPU: simulate, sync, upload per-frame buffers) → `onPreRender` (record offscreen
passes into the frame's command buffer) → swapchain render pass (`ssfrRenderer_` composite + ImGui).

## Owners

| Resource | Owner | Created | Destroyed | Swapchain resize |
|---|---|---|---|---|
| `hdrScene_` (`VulkanOffscreen`, RGBA16F + depth) and `hdrSampler_` | `FluidApp` | `createHdrTargets()` from `onSwapChainCreated()` (and once in `onInit()`) | `destroyHdrTargets()` from `onSwapChainDestroying()` and `onCleanup()` | Rebuilt only when the extent changed (`create()` is idempotent: it destroys first). The render-pass handle changes, so `onSwapChainCreated()` hands it to `rigidGltfRenderer_` / `softGltfRenderer_` (they build pipelines lazily per body) and re-points `ssfrRenderer_.setSceneInput()` at the new views. Already-built pipelines stay valid through render-pass compatibility (same formats / sample counts). |
| `shadowPass_` (`ShadowMapPass`, 2048² depth) | `FluidApp` | `onInit()` | `onCleanup()`, **after** the sub-renderers destroyed their shadow pipelines (those reference its render pass) | Fixed size; not touched. |
| `ssfrRenderer_` and its offscreen targets (`SSFROffscreenSet`) | `FluidApp` as the only `VkAppBase` sub-renderer | `onInit()` | `VkAppBase::onCleanup()` | `resize()` rebuilds the targets. |
| `hdrRenderers_` (background glTF, fluid points, rigid / soft glTF and wire, hair, volume, mesh, combustible, flame, flame debug, cloud volume; list set in the constructor) | `FluidApp` members; **not** `VkAppBase` sub-renderers | `onInit()`: `onInit(ctx, pool, hdrScene_.getRenderPass(), MAX_FRAMES_IN_FLIGHT)` on each, right after the first `createHdrTargets()` | `onCleanup()` calls `onCleanup(device)` on each, after `VkAppBase::onCleanup()` | Pipelines survive; the flame and cloud volume renderers call `resize()` for their own targets. |
| GPU_CSPH particle buffers (`CSPHSolverVk`, held in `FluidWorld::gpuSolver_`) | `FluidWorld` | `FluidWorld::reset()` (`createGPUCSPH()`, needs `setVulkanContext()`) | `FluidWorld::clear()` / `reset()`, and `releaseGpuResources()` | Not tied to the swapchain. |
| glTF documents and environment textures | `RenderBackground` (keeps the `GltfDocument` alive for in-flight frames) | `LoadRenderBackground` / `SetEnvironment` | `ClearRenderBackground`; `newScene()` calls `RenderBackground::clearBackground()` | Not tied to the swapchain. |

## Order rules

1. **`FluidWorld::releaseGpuResources()` first.** `FluidApp::onCleanup()` calls it before
   `VkAppBase::onCleanup()`, because `CSPHSolverVk` frees device objects and the `VulkanContext` is
   destroyed right after `onCleanup()` returns.
2. `VkAppBase::onCleanup()` (SSFR) → the `hdrRenderers_` → `shadowPass_` → `destroyHdrTargets()`.
   The device is idle (`cleanup()` waited).
3. `onSwapChainDestroying()` runs with the device idle, so destroying `hdrScene_` there is safe.
4. `run()` calls `onInit()` then `onSwapChainCreated()` back to back at the same size; the extent
   check in `onSwapChainCreated()` skips the second build so the pipelines `onInit()` just made are
   not left pointing at a freed render pass.

## Non-owning references that cross a lifetime boundary

- **GPU_CSPH position buffer** → `fluidRenderer_.setDirectGpuBuffer()` and
  `ssfrRenderer_.setParticleBuffer()` hold the raw `VkBuffer`. `Reset` rebuilds the solver (new
  buffer), so the renderers must be re-pointed: the dispatcher's
  `onWorldChanged_` callback runs `syncGpuCsphBufferToRenderer()` after every command that changes
  the world, and the per-frame sync does it while the fluid runs. When the solver is gone
  `syncGpuCsphBufferToRenderer()` clears both references. The compute-written buffer is used as
  vertex input, so `onPreRender()` records a `SHADER_WRITE → VERTEX_ATTRIBUTE_READ` barrier first.
- **`ShadowMapPass` views / sampler** → taken by value-handle by the glTF renderers; valid until
  `shadowPass_.destroy()`.
- **`hdrScene_` color / depth views** → held by `ssfrRenderer_` (scene input) and the flame haze /
  PBVR passes; valid until the next `createHdrTargets()`, which is why the resize path re-points
  them.
- **`GltfDocument`** → `GltfSceneRenderer::loadDocument()` stores a pointer into it, so the
  document must outlive in-flight frames (hence `RenderBackground` owns it).

## Not yet verified

- Resizing the window with every domain populated (flame PBVR + haze + cloud volume) on the GPU
  after a resize has only been exercised through the scenario captures, not by an automated
  resize test.
