# Cross-platform CMake build for the Physics module: the core library (PhysicsCore), the
# CPU-side world / conversion code shared by the viewer and the tests (PhysicsWorldCore), the
# GoogleTest suite (PhysicsTest), the headless examples, and PhysicsView (Vulkan + ImGui app).
#
# CMake is the only build system (the .vcxproj files are gone). Shared library targets
# (MathCore, SpaceCore, VolumeCore, NumericsCore, VkAppBaseCore, ...) are defined once in
# CGLib/cmake (PhantomCoreLibs.cmake, PhantomVulkanApp.cmake); this file calls them. It works
# standalone (`cmake -S Physics -B ...`) and identically when add_subdirectory()'d from the
# Phantom root CMakeLists.txt.
#
# Targets and what each needs:
#   PhysicsCore       static, CPU only. Public contract for CGApp / PyFluid / examples.
#   PhysicsWorldCore  static, CPU only, internal (not installed, not linked by CGApp). The
#                     UI- and Vulkan-free parts of PhysicsView: the worlds (rigid, flame, cloud,
#                     hair), glTF/wire conversion, command dispatchers without rendering.
#                     PhysicsView and PhysicsTest link the same objects.
#   PhysicsTest       needs GoogleTest only (so it builds with no Vulkan SDK).
#   PhysicsView       needs the Vulkan headers + loader and GLFW; skipped with a warning
#                     otherwise. FluidRendererCore and FluidGPUVkCore are leaf libraries only
#                     PhysicsView links.
#   examples/         headless programs linking PhysicsCore only (examples/README.md).
#
# Usage (from WSL, in a Linux-native checkout -- avoid /mnt/c/... for speed):
#   cmake -S Physics -B Physics/build_linux #       -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang #       -DCMAKE_BUILD_TYPE=Debug
#   cmake --build Physics/build_linux -j
#   ctest --test-dir Physics/build_linux --output-on-failure
#
# Flame (folded in from the former FlameView) is self-contained: point-sprite pipelines inside
# PhysicsView, independent of ISPHSolver and rigid/soft dynamics, optionally coupled to static
# finite-fuel solids.

cmake_minimum_required(VERSION 3.20)

project(PhysicsCoreLinux LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(PHYSICS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(CGLIB_ROOT ${REPO_ROOT}/CGLib)

if(MSVC)
    add_compile_options(/utf-8)
    set(PHANTOM_WARN_FLAGS /W3)
else()
    set(PHANTOM_WARN_FLAGS -Wall)
endif()

list(APPEND CMAKE_MODULE_PATH ${REPO_ROOT}/cmake)
include(PhantomGTest)
include(PhantomCoreLibs)
include(PhantomVulkanApp)

phantom_add_math_core()
phantom_add_space_core()
phantom_add_volume_core()
phantom_add_numerics_core()

# CONFIGURE_DEPENDS so adding/removing a .cpp (e.g. splitting a boundary class
# out into its own file) is picked up on the next build without a manual
# `cmake <builddir>` reconfigure -- otherwise a stale object of a rewritten
# source can linger and cause "unresolved external symbol" at link time.
file(GLOB PHYSICS_SOURCES CONFIGURE_DEPENDS ${PHYSICS_ROOT}/Physics/*.cpp)
list(FILTER PHYSICS_SOURCES EXCLUDE REGEX "pch\\.cpp$")

add_library(PhysicsCore STATIC ${PHYSICS_SOURCES})
target_include_directories(PhysicsCore PRIVATE ${REPO_ROOT})
target_link_libraries(PhysicsCore PUBLIC MathCore SpaceCore VolumeCore NumericsCore)

# OpenMP is an implementation detail of the solvers (`#pragma omp` lives in .cpp files only, no
# public header uses it), so it is not part of PhysicsCore's interface. Targets that use OpenMP
# themselves (PhysicsTest's FlameSolverTest) link it explicitly below.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(PhysicsCore PRIVATE OpenMP::OpenMP_CXX)
endif()

target_compile_options(PhysicsCore PRIVATE ${PHANTOM_WARN_FLAGS})

# ---------------------------------------------------------------------------
# PhysicsWorldCore -- the CPU-side part of PhysicsView, compiled once and linked by both
# PhysicsView and PhysicsTest (previously PhysicsTest recompiled a hand-picked subset of
# PhysicsView's .cpp files, which then had to be kept in sync by hand). No Vulkan, ImGui or GLFW:
# the worlds, glTF / wire data builders and command dispatchers that do not draw.
# Add a PhysicsView .cpp here (not to PhysicsView below) when it stays free of those APIs.
# ---------------------------------------------------------------------------
add_library(PhysicsWorldCore STATIC
    ${PHYSICS_ROOT}/PhysicsView/PrimitiveGltf.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftMeshGltf.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyWireData.cpp
    ${PHYSICS_ROOT}/PhysicsView/CombustibleGltf.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameBlackbody.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/CloudWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/HairWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/HairCommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/RigidBodyWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/SceneComponentRegistry.cpp)
target_include_directories(PhysicsWorldCore PRIVATE
    ${REPO_ROOT}
    ${CGLIB_ROOT}/ThirdParty/glm-0.9.9.8)
target_link_libraries(PhysicsWorldCore PUBLIC PhysicsCore)
if(OpenMP_CXX_FOUND)
    target_link_libraries(PhysicsWorldCore PRIVATE OpenMP::OpenMP_CXX)
endif()
target_compile_options(PhysicsWorldCore PRIVATE ${PHANTOM_WARN_FLAGS})
target_compile_features(PhysicsWorldCore PUBLIC cxx_std_20)

# ---------------------------------------------------------------------------
# examples/ -- headless (no Vulkan/ImGui) sample programs linking only
# PhysicsCore. An onboarding entry point for the open-source release; see
# Physics/examples/README.md. Independent of the Vulkan/GLFW discovery below.
# ---------------------------------------------------------------------------
option(PHYSICS_BUILD_EXAMPLES "Build the headless Physics/examples/ programs" ON)
if(PHYSICS_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------
# PhysicsTest (GoogleTest). The whole module is C++20: the standalone build sets
# CMAKE_CXX_STANDARD 20 above, the Phantom root build does the same, and CGLib's *Core targets
# propagate cxx_std_20. The per-target properties below only restate it.
# ---------------------------------------------------------------------------

phantom_find_gtest()

if(PHANTOM_GTEST_FOUND)
    enable_testing()

    file(GLOB PHYSICSTEST_SOURCES CONFIGURE_DEPENDS ${PHYSICS_ROOT}/PhysicsTest/*.cpp)
    list(FILTER PHYSICSTEST_SOURCES EXCLUDE REGEX "pch\\.cpp$")
    # Long-running WCSPH scene/stability checks belong to PhysicsView's
    # scenario suite, not the component-level GoogleTest executable.
    list(FILTER PHYSICSTEST_SOURCES EXCLUDE REGEX
        "(FluidPoolStabilityTest|WaterSphereEmitterStabilityTest|SPHScaleInvarianceTest)\\.cpp$")

    # The CPU-side PhysicsView code under test (glTF builders, worlds, hair dispatcher) comes
    # from PhysicsWorldCore, the same objects PhysicsView links.
    # cglib_add_test (CGLib/cmake/PhantomGTest.cmake) creates the executable, puts REPO_ROOT on
    # the include path, links gtest_main and registers it with CTest.
    cglib_add_test(PhysicsTest
        SOURCES ${PHYSICSTEST_SOURCES}
        LINK PhysicsWorldCore PhysicsCore
        INCLUDES ${PHYSICS_ROOT}/PhysicsTest ${CGLIB_ROOT}/ThirdParty/glm-0.9.9.8)
    set_target_properties(PhysicsTest PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )
    if(OpenMP_CXX_FOUND)
        target_link_libraries(PhysicsTest PRIVATE OpenMP::OpenMP_CXX)   # FlameSolverTest uses OpenMP
    endif()
else()
    message(WARNING "GTest not found -- skipping PhysicsTest target")
endif()

# ---------------------------------------------------------------------------
# PhysicsView (Vulkan/ImGui app).
# ---------------------------------------------------------------------------

phantom_find_vulkan_headers()

if(PHANTOM_VULKAN_INCLUDE_DIR)

phantom_find_vulkan_loader()
phantom_find_glfw_loader()

if(PHANTOM_VULKAN_LIBRARY AND PHANTOM_GLFW_LIBRARY)

# Build runtime SPIR-V assets from their GLSL sources. phantom_add_runtime_shaders()
# now lives in cmake/PhantomVulkanApp.cmake (shared with PointCloud/PointCloudView
# + GSView); the .spv files are intentionally Git-ignored, so a clean checkout
# needs glslc rather than just a copy of the GLSL source dir.

phantom_add_graphics_core()
phantom_add_file_core()
phantom_add_vkappbase_core()
phantom_add_vkrenderer_core()

# GltfSceneRenderer (CGLib/GltfRenderer) -- PhysicsView hosts one instance as a
# glTF background/set renderer on the shared FluidRenderer camera
# (docs/todo/PLAN_physicsview_gltf_rendering.md). Pulls in AnimationCore /
# FileCore / VulkanGraphicsCore / GraphicsCore transitively; listed explicitly
# in target_link_libraries below to match the plan's wiring section.
phantom_add_gltfrenderer_core()
phantom_add_volumeraymarch_core()

# --- FluidRenderer (Screen Space Fluid Rendering) -- PhysicsView only ---

add_library(FluidRendererCore STATIC
    ${PHYSICS_ROOT}/FluidRenderer/ParticleDepthRenderer.cpp
    ${PHYSICS_ROOT}/FluidRenderer/SSFROffscreenSet.cpp
    ${PHYSICS_ROOT}/FluidRenderer/SSFRPipeline.cpp
    ${PHYSICS_ROOT}/FluidRenderer/SSThicknessRenderer.cpp
    ${PHYSICS_ROOT}/FluidRenderer/BilateralFilter.cpp
    ${PHYSICS_ROOT}/FluidRenderer/SSReflectionRenderer.cpp
    ${PHYSICS_ROOT}/FluidRenderer/SSRefractionRenderer.cpp
    ${PHYSICS_ROOT}/FluidRenderer/SSFluidRenderer.cpp
)
target_include_directories(FluidRendererCore PRIVATE
    ${CGLIB_ROOT}/ThirdParty/VulkanMemoryAllocator
    ${CGLIB_ROOT}/ThirdParty/imgui
)
target_link_libraries(FluidRendererCore PRIVATE VulkanGraphicsCore)
target_compile_options(FluidRendererCore PRIVATE ${PHANTOM_WARN_FLAGS})
target_compile_features(FluidRendererCore PRIVATE cxx_std_20)

# --- Fluid_GPU_Vk (GPU Compute CSPH) -- PhysicsView only ---
#
# CSPHSolverVk.cpp loads precompiled .spv files at runtime via
# VulkanSPVResolver.h, same as every other Vulkan app/library in this repo
# (see Fluid_GPU_Vk/Shader/compile_shaders.bat) -- no libshaderc dependency.

add_library(FluidGPUVkCore STATIC
    ${PHYSICS_ROOT}/Fluid_GPU_Vk/CSPHParticleBufferVk.cpp
    ${PHYSICS_ROOT}/Fluid_GPU_Vk/CSPHGridBufferVk.cpp
    ${PHYSICS_ROOT}/Fluid_GPU_Vk/CSPHSolverVk.cpp
)
target_link_libraries(FluidGPUVkCore PRIVATE VulkanGraphicsCore)
target_compile_options(FluidGPUVkCore PRIVATE ${PHANTOM_WARN_FLAGS})
target_compile_features(FluidGPUVkCore PRIVATE cxx_std_20)

# --- PhysicsView (FluidApp : VkAppBase) -- fluid/rigid/soft-body standalone viewer ---

add_executable(PhysicsView
    ${PHYSICS_ROOT}/PhysicsView/main.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidApp.cpp
    ${PHYSICS_ROOT}/PhysicsView/SSFRSettings.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidPipeline.cpp
    ${PHYSICS_ROOT}/PhysicsView/Camera.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidPLYWriter.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidVolumeConverter.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidMeshConverter.cpp
    ${PHYSICS_ROOT}/PhysicsView/VolumeRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidMeshRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/RenderBackground.cpp
    ${PHYSICS_ROOT}/PhysicsView/GltfBodyRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/GltfSoftRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/ObjectListPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/CommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/RigidBodyWireRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/RigidBodyCommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyWireRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyCommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/CombustibleRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameCommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlamePointPipeline.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameFullscreenPass.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameHazePass.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlamePBVRPass.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameStreamBuffer.cpp
    ${PHYSICS_ROOT}/PhysicsView/CloudCommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/CloudVolumeRenderer.cpp
)
target_include_directories(PhysicsView PRIVATE
    ${CGLIB_ROOT}/ThirdParty/VulkanMemoryAllocator
    ${CGLIB_ROOT}/UIWidgets
    ${PHYSICS_ROOT}/FluidRenderer
    ${PHYSICS_ROOT}/Fluid_GPU_Vk
    ${PHYSICS_ROOT}/Physics
)
target_link_libraries(PhysicsView PRIVATE
    PhysicsWorldCore
    PhysicsCore
    FluidRendererCore
    FluidGPUVkCore
    FileCore
    VkAppBaseCore
    VkRendererCore
    GltfRendererCore
    AnimationCore
    VolumeRaymarch
    # VulkanGraphicsCore must precede GraphicsCore on the link line: GltfReader.cpp
    # (GltfRendererCore) pulls stbi_load_from_memory, and both VulkanCubeMap.cpp
    # (VulkanGraphicsCore, already linked for SSFluidRenderer's cube maps) and
    # ImageFileReader.cpp (GraphicsCore) embed a full STB_IMAGE_IMPLEMENTATION.
    # Resolving stbi_* from VulkanCubeMap.cpp first keeps ImageFileReader.cpp.obj
    # out of the image, avoiding a duplicate stb_image (LNK2005). Nothing here
    # references GraphicsCore's ImageFileReader/ImageFileWriter classes directly.
    # (PLAN_physicsview_gltf_rendering.md Phase 1 / risk table item 8.)
    VulkanGraphicsCore
    GraphicsCore
    ${PHANTOM_VULKAN_LIBRARY}
    ${PHANTOM_GLFW_LIBRARY}
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(PhysicsView PRIVATE OpenMP::OpenMP_CXX)
endif()
target_compile_options(PhysicsView PRIVATE ${PHANTOM_WARN_FLAGS})
target_compile_features(PhysicsView PRIVATE cxx_std_20)

# PhysicsView loads shaders from three source trees, one per library it links,
# all flattened into a single `shaders/` directory next to the executable
# (VulkanSPVResolver resolves every name against that one directory):
#   - PhysicsView/shaders      -- the viewer's own point/line/triangle pipelines
#   - Fluid_GPU_Vk/Shader      -- GPU_CSPH compute shaders (CSPHSolverVk)
#   - FluidRenderer/shaders    -- SSFR + skybox (SSFluidRenderer, FluidRendererCore)
# The FluidRenderer copy was missing, so a CMake-built PhysicsView.exe crashed on
# startup (vkCreateShaderModule with codeSize 0) rather than merely losing SSFR --
# SSFluidRenderer is constructed unconditionally. Keep this list in sync with
# PhysicsView's target_link_libraries above: linking another Vulkan renderer
# library means adding its shader directory here too.
# PhysicsView/shaders also carries the six flame_*.{vert,frag} sources (folded
# in with the former standalone FlameView) -- FlameRenderer's three pipelines
# load them by name via VulkanSPVResolver from the same flattened shaders/ dir.
# It likewise carries gltf.{vert,frag} (copied verbatim from CGLib/GltfViewer/
# shaders -- the SDR/in-shader-tonemap variant, same as Universe uses) for the
# GltfSceneRenderer background pass (PLAN_physicsview_gltf_rendering.md Phase 0).
phantom_add_runtime_shaders(PhysicsView
    ${PHYSICS_ROOT}/PhysicsView/shaders
    ${PHYSICS_ROOT}/Fluid_GPU_Vk/Shader
    ${PHYSICS_ROOT}/FluidRenderer/shaders
    ${CGLIB_ROOT}/Volume/VolumeRaymarch/shaders)

# Default cubemap used by SSFR when the viewer starts. Generate it beside the
# executable so binary images do not need to be stored in source control.
option(PHYSICS_GENERATE_DEFAULT_ENVMAP "Generate the default SSFR cubemap after building PhysicsView (needs Windows PowerShell + System.Drawing)" ON)
if(PHYSICS_GENERATE_DEFAULT_ENVMAP)
    find_program(PHANTOM_POWERSHELL_EXECUTABLE NAMES pwsh powershell REQUIRED)
    add_custom_command(TARGET PhysicsView POST_BUILD
        COMMAND "${PHANTOM_POWERSHELL_EXECUTABLE}"
            -NoProfile -ExecutionPolicy Bypass
            -File "${PHYSICS_ROOT}/FluidRenderer/assets/default_envmap/generate_indoor_envmap.ps1"
            -OutputDir "$<TARGET_FILE_DIR:PhysicsView>/envmap"
        VERBATIM)
endif()

else()
    message(WARNING "Physics: Vulkan and/or GLFW loader not found -- PhysicsView is skipped (Physics/PhysicsTest still build). Pass -DVULKAN_LIBRARY=... -DGLFW_LIBRARY=... to enable it.")
endif()

else()
    message(WARNING "Physics: vulkan/vulkan.h not found -- PhysicsView is skipped. Pass -DVULKAN_INCLUDE_DIR=/path/to/include to enable it (on Windows, check $ENV{VULKAN_SDK} is set).")
endif()

# Precompiled headers (cmake/PhantomPch.cmake). Physics/pch.h and PhysicsTest/pch.h hold
# only the STL / gtest, so their lists cover every TU; PhysicsView/ and Fluid_GPU_Vk/
# keep their own pch.h (it sets GLM_FORCE_* macros) for the files that include it.
include(PhantomPch)
phantom_use_pch_headers(PhysicsCore ${PHANTOM_PCH_STL})
phantom_use_pch_headers(PhysicsTest ${PHANTOM_PCH_GTEST})
phantom_use_pch(PhysicsWorldCore ${PHYSICS_ROOT}/PhysicsView/pch.h)
phantom_use_pch(FluidGPUVkCore ${PHYSICS_ROOT}/Fluid_GPU_Vk/pch.h)
phantom_use_pch(PhysicsView    ${PHYSICS_ROOT}/PhysicsView/pch.h)
