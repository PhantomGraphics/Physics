# Cross-platform CMake build for the Physics core library (Phantom::Physics),
# its GoogleTest suite (PhysicsTest), and PhysicsView/FlameView (Vulkan apps).
#
# This is a standalone CMake build, separate from the MSBuild/.vcxproj-based
# Windows build (Physics/Physics/Physics.vcxproj + Physics/PhysicsTest/
# PhysicsTest.vcxproj, driven by Phantom2026.sln). MathCore/SpaceCore/
# VolumeCore/NumericsCore/VkAppBaseCore/etc. are now defined once in
# ../cmake/PhantomCoreLibs.cmake and ../cmake/PhantomVulkanApp.cmake
# (Phase 4 §5.4 item 1 dedup, internal design notes)
# -- this file just calls them (PhysicsCore itself links against those
# instead of bundling their raw sources, same behavior, cleaner) and adds
# PhysicsTest/PhysicsView/FlameView on top, so it also works standalone
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
# FlameView.vcxproj's real <ProjectReference> list is much narrower: Math,
# VulkanGraphics, Space, VkAppBase, Physics -- no VkRenderer, no UIWidgets/
# Graphics reference of its own, no FluidRenderer/Fluid_GPU_Vk/File/Numerics
# -- FlameRenderer/FlamePipeline/FlameSmokePipeline/FlamePBVRPipeline are
# self-contained pipelines, not built atop VkRenderer or SSFR (matches
# Physics/CLAUDE.md's description of Flame as an intentionally independent,
# narrower subsystem).

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
# PhysicsView + FlameView (Vulkan/ImGui apps) -- Phase 2.
# ---------------------------------------------------------------------------

phantom_find_vulkan_headers()

if(PHANTOM_VULKAN_INCLUDE_DIR)

phantom_find_vulkan_loader()
phantom_find_glfw_loader()

if(PHANTOM_VULKAN_LIBRARY AND PHANTOM_GLFW_LIBRARY)

# Build runtime SPIR-V assets from their GLSL sources. The .spv files are
# intentionally ignored by Git, so copying the source shader directories is
# insufficient on a clean checkout (and leads to zero-sized shader modules at
# runtime). Vulkan SDK installations expose glslc in Bin/.
find_program(PHANTOM_GLSLC_EXECUTABLE
    NAMES glslc glslc.exe
    HINTS "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/bin")
if(NOT PHANTOM_GLSLC_EXECUTABLE)
    message(FATAL_ERROR
        "PhysicsView/FlameView require glslc to compile runtime shaders. "
        "Install the Vulkan SDK or set VULKAN_SDK.")
endif()

function(phantom_add_runtime_shaders target)
    set(shader_output_dir "${CMAKE_CURRENT_BINARY_DIR}/${target}_shaders")
    set(shader_outputs)
    foreach(shader_dir IN LISTS ARGN)
        file(GLOB shader_sources CONFIGURE_DEPENDS
            "${shader_dir}/*.vert" "${shader_dir}/*.frag" "${shader_dir}/*.comp")
        foreach(shader_source IN LISTS shader_sources)
            get_filename_component(shader_name "${shader_source}" NAME)
            set(shader_output "${shader_output_dir}/${shader_name}.spv")
            add_custom_command(
                OUTPUT "${shader_output}"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${shader_output_dir}"
                COMMAND "${PHANTOM_GLSLC_EXECUTABLE}" "${shader_source}" -o "${shader_output}"
                DEPENDS "${shader_source}"
                COMMENT "Compiling ${shader_name}"
                VERBATIM)
            list(APPEND shader_outputs "${shader_output}")
        endforeach()
    endforeach()

    add_custom_target(${target}Shaders DEPENDS ${shader_outputs})
    add_dependencies(${target} ${target}Shaders)
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${shader_output_dir}" "$<TARGET_FILE_DIR:${target}>/shaders")
endfunction()

phantom_add_graphics_core()
phantom_add_file_core()
phantom_add_vkappbase_core()
phantom_add_vkrenderer_core()

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
    ${PHYSICS_ROOT}/PhysicsView/SSFRPanel.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidVolumeConverter.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidMeshConverter.cpp
    ${PHYSICS_ROOT}/PhysicsView/VolumeRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidMeshRenderer.cpp
    ${PHYSICS_ROOT}/PhysicsView/FluidVolumeConvertPanel.cpp
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

# --- FlameView (FlameApp : VkAppBase) -- minimal flame-SPH standalone viewer,
# intentionally independent of RigidBodyWorld/VkRenderer/FluidRenderer (see
# Physics/CLAUDE.md's Flame section) ---

add_executable(FlameView
    ${PHYSICS_ROOT}/FlameView/main.cpp
    ${PHYSICS_ROOT}/FlameView/FlameApp.cpp
    ${PHYSICS_ROOT}/FlameView/FlameRenderer.cpp
    ${PHYSICS_ROOT}/FlameView/FlamePipeline.cpp
    ${PHYSICS_ROOT}/FlameView/FlameSmokePipeline.cpp
    ${PHYSICS_ROOT}/FlameView/FlamePBVRPipeline.cpp
)
target_include_directories(FlameView PRIVATE
    ${CGLIB_ROOT}/UIWidgets
    ${PHYSICS_ROOT}/Physics
)
target_link_libraries(FlameView PRIVATE
    PhysicsCore
    VkAppBaseCore
    ${PHANTOM_VULKAN_LIBRARY}
    ${PHANTOM_GLFW_LIBRARY}
)
if(OpenMP_CXX_FOUND)
    target_link_libraries(FlameView PRIVATE OpenMP::OpenMP_CXX)
endif()
target_compile_options(FlameView PRIVATE ${PHANTOM_WARN_FLAGS})
target_compile_features(FlameView PRIVATE cxx_std_20)

phantom_add_runtime_shaders(FlameView ${PHYSICS_ROOT}/FlameView/shaders)

else()
    message(WARNING "Physics: Vulkan and/or GLFW loader not found -- PhysicsView/FlameView are skipped (Physics/PhysicsTest still build). Pass -DVULKAN_LIBRARY=... -DGLFW_LIBRARY=... to enable them.")
endif()

else()
    message(WARNING "Physics: vulkan/vulkan.h not found -- PhysicsView/FlameView are skipped. Pass -DVULKAN_INCLUDE_DIR=/path/to/include to enable them (on Windows, check $ENV{VULKAN_SDK} is set).")
endif()
