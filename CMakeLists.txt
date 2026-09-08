# Cross-platform CMake build for the Physics core library (Phantom::Physics),
# its GoogleTest suite (PhysicsTest), and PhysicsView (Vulkan app).
#
# This is a standalone CMake build, separate from the MSBuild/.vcxproj-based
# Windows build (Physics/Physics/Physics.vcxproj + Physics/PhysicsTest/
# PhysicsTest.vcxproj, driven by Phantom2026.sln). MathCore/SpaceCore/
# VolumeCore/NumericsCore/VkAppBaseCore/etc. are now defined once in
# ../cmake/PhantomCoreLibs.cmake and ../cmake/PhantomVulkanApp.cmake
# (Phase 4 §5.4 item 1 dedup, internal design notes)
# -- this file just calls them (PhysicsCore itself links against those
# instead of bundling their raw sources, same behavior, cleaner) and adds
# PhysicsTest/PhysicsView on top, so it also works standalone
# (`cmake -S Physics -B ...`) exactly as before, and identically when
# add_subdirectory()'d from the root CMakeLists.txt.
#
# Usage (from WSL, in a Linux-native checkout -- avoid /mnt/c/... for speed):
#   cmake -S Physics -B Physics/build_linux \
#       -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang \
#       -DCMAKE_BUILD_TYPE=Debug
#   cmake --build Physics/build_linux -j
#   ctest --test-dir Physics/build_linux --output-on-failure
#
# PhysicsView.vcxproj's real <ProjectReference> list (verified, not the plan
# doc's dependency sketch) is: Math, VulkanGraphics, Space, VkAppBase,
# VkRenderer, UIWidgets, FluidRenderer, Physics, Numerics, Fluid_GPU_Vk,
# Volume, File -- no Phantom::Scene. FluidRenderer and Fluid_GPU_Vk have NO
# <ProjectReference> of their own (header-only include paths onto Math/
# VulkanGraphics/imgui/glm/VMA) -- they're leaf static libs PhysicsView links.
#
# FlameRenderer/FlamePipeline/FlameSmokePipeline/FlamePBVRPipeline (folded in
# from the former standalone FlameView) are self-contained point-sprite
# pipelines, not built atop VkRenderer or SSFR -- they compile straight into
# PhysicsView now, and Flame stays an intentionally uncoupled subsystem (matches
# Physics/CLAUDE.md's Flame section).

cmake_minimum_required(VERSION 3.20)

project(PhysicsCoreLinux LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
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

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(PhysicsCore PUBLIC OpenMP::OpenMP_CXX)
endif()

target_compile_options(PhysicsCore PRIVATE ${PHANTOM_WARN_FLAGS})

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
# PhysicsTest (GoogleTest) -- PhysicsTest.vcxproj explicitly overrides
# <LanguageStandard> to stdcpp20 (unlike PhysicsCore's Physics.vcxproj, which
# doesn't and falls back to BuildConfig.props' repo-wide C++17 default) --
# CXX_STANDARD is set per-target below rather than changing CMAKE_CXX_STANDARD
# globally, so PhysicsCore itself stays C++17 exactly as before.
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

    add_executable(PhysicsTest ${PHYSICSTEST_SOURCES})
    set_target_properties(PhysicsTest PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
    )

    target_include_directories(PhysicsTest PRIVATE
        ${REPO_ROOT}
        ${PHYSICS_ROOT}/PhysicsTest
    )

    target_link_libraries(PhysicsTest PRIVATE
        PhysicsCore
        GTest::gtest
        GTest::gtest_main
    )

    target_compile_options(PhysicsTest PRIVATE ${PHANTOM_WARN_FLAGS})

    include(GoogleTest)
    gtest_discover_tests(PhysicsTest)
else()
    message(WARNING "GTest not found -- skipping PhysicsTest target")
endif()

# ---------------------------------------------------------------------------
# PhysicsView (Vulkan/ImGui app) -- Phase 2.
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
    ${PHYSICS_ROOT}/PhysicsView/FluidPipeline.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidPLYWriter.cpp
    ${PHYSICS_ROOT}/PhysicsView/ControlPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidStatusView.cpp
    ${PHYSICS_ROOT}/PhysicsView/SSFRPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidVolumeConverter.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidMeshConverter.cpp
    ${PHYSICS_ROOT}/PhysicsView/VolumeRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidMeshRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidVolumeConvertPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/ControlPanelHost.cpp
    ${PHYSICS_ROOT}/PhysicsView/RenderBackground.cpp
    ${PHYSICS_ROOT}/PhysicsView/RenderingPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/SceneComponentRegistry.cpp
    ${PHYSICS_ROOT}/PhysicsView/ObjectListPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/CommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/SSFRTestPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/RigidBodyWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/RigidBodyWireRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/RigidBodyControlPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/RigidBodyCommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyWireRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyControlPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/SoftBodyCommandDispatcher.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameWorld.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameControlPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlamePipeline.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlameSmokePipeline.cpp
    ${PHYSICS_ROOT}/PhysicsView/FlamePBVRPipeline.cpp
)
target_include_directories(PhysicsView PRIVATE
    ${CGLIB_ROOT}/ThirdParty/VulkanMemoryAllocator
    ${CGLIB_ROOT}/UIWidgets
    ${PHYSICS_ROOT}/FluidRenderer
    ${PHYSICS_ROOT}/Fluid_GPU_Vk
    ${PHYSICS_ROOT}/Physics
)
target_link_libraries(PhysicsView PRIVATE
    PhysicsCore
    FluidRendererCore
    FluidGPUVkCore
    FileCore
    VkAppBaseCore
    VkRendererCore
    GltfRendererCore
    AnimationCore
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
    ${PHYSICS_ROOT}/FluidRenderer/shaders)

# Default cubemap used by SSFR when the viewer starts. Generate it beside the
# executable so binary images do not need to be stored in source control.
find_program(PHANTOM_POWERSHELL_EXECUTABLE NAMES pwsh powershell REQUIRED)
add_custom_command(TARGET PhysicsView POST_BUILD
    COMMAND "${PHANTOM_POWERSHELL_EXECUTABLE}"
        -NoProfile -ExecutionPolicy Bypass
        -File "${PHYSICS_ROOT}/FluidRenderer/assets/default_envmap/generate_indoor_envmap.ps1"
        -OutputDir "$<TARGET_FILE_DIR:PhysicsView>/envmap"
    VERBATIM)

# FlameView (the former standalone flame-SPH viewer) was folded into PhysicsView
# on 2026-09-08: FlameRenderer/FlamePipeline/FlameSmokePipeline/FlamePBVRPipeline
# and the flame_*.{vert,frag} shaders moved under PhysicsView/, and FlameApp's
# scene/UI became PhysicsView's FlameWorld + FlameControlPanel (ControlPage::Flame).
# Flame is still an intentionally uncoupled subsystem (FlameSolver does not
# implement ISPHSolver) -- see Physics/CLAUDE.md's Flame section.

else()
    message(WARNING "Physics: Vulkan and/or GLFW loader not found -- PhysicsView is skipped (Physics/PhysicsTest still build). Pass -DVULKAN_LIBRARY=... -DGLFW_LIBRARY=... to enable it.")
endif()

else()
    message(WARNING "Physics: vulkan/vulkan.h not found -- PhysicsView is skipped. Pass -DVULKAN_INCLUDE_DIR=/path/to/include to enable it (on Windows, check $ENV{VULKAN_SDK} is set).")
endif()
