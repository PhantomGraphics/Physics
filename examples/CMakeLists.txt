# Physics/examples -- headless (no Vulkan, no ImGui) sample programs that link
# only against PhysicsCore. See examples/README.md for the walkthrough.
#
# Two ways to build these:
#
#  1. As part of the normal build -- Physics/CMakeLists.txt does
#     add_subdirectory(examples) when PHYSICS_BUILD_EXAMPLES is ON (the
#     default), so `cmake --preset windows-debug` already produces them under
#     build/windows-debug/Physics/examples/.
#
#  2. Standalone, pointing at a checkout of the Phantom superproject:
#         cmake -S Physics/examples -B build/examples -DCMAKE_BUILD_TYPE=Release
#         cmake --build build/examples
#     This path rebuilds a private copy of PhysicsCore from source.

cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED PROJECT_NAME)
    project(PhantomPhysicsExamples LANGUAGES CXX)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Locate PhysicsCore: reuse the in-tree target if we were add_subdirectory'd,
# otherwise build a standalone copy the same way Physics/CMakeLists.txt does.
# ---------------------------------------------------------------------------
if(NOT TARGET PhysicsCore)
    get_filename_component(_examples_dir "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
    set(PHYSICS_ROOT "${_examples_dir}/..")
    get_filename_component(PHYSICS_ROOT "${PHYSICS_ROOT}" ABSOLUTE)
    get_filename_component(REPO_ROOT "${PHYSICS_ROOT}/.." ABSOLUTE)
    set(CGLIB_ROOT "${REPO_ROOT}/CGLib")

    if(NOT EXISTS "${REPO_ROOT}/cmake/PhantomCoreLibs.cmake")
        message(FATAL_ERROR
            "Standalone examples build needs the Phantom superproject layout "
            "(expected ${REPO_ROOT}/cmake/PhantomCoreLibs.cmake). Clone Phantom "
            "with --recurse-submodules and build from there, or use the "
            "PHYSICS_BUILD_EXAMPLES path in Physics/CMakeLists.txt.")
    endif()

    if(MSVC)
        add_compile_options(/utf-8)
        set(PHANTOM_WARN_FLAGS /W3)
    else()
        set(PHANTOM_WARN_FLAGS -Wall)
    endif()

    list(APPEND CMAKE_MODULE_PATH "${REPO_ROOT}/cmake")
    include(PhantomCoreLibs)
    phantom_add_math_core()
    phantom_add_space_core()
    phantom_add_volume_core()
    phantom_add_numerics_core()

    file(GLOB _physics_sources CONFIGURE_DEPENDS "${PHYSICS_ROOT}/Physics/*.cpp")
    list(FILTER _physics_sources EXCLUDE REGEX "pch\\.cpp$")
    add_library(PhysicsCore STATIC ${_physics_sources})
    target_include_directories(PhysicsCore PRIVATE "${REPO_ROOT}")
    target_link_libraries(PhysicsCore PUBLIC MathCore SpaceCore VolumeCore NumericsCore)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(PhysicsCore PUBLIC OpenMP::OpenMP_CXX)
    endif()
    target_compile_options(PhysicsCore PRIVATE ${PHANTOM_WARN_FLAGS})
else()
    get_filename_component(PHYSICS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
    get_filename_component(REPO_ROOT "${PHYSICS_ROOT}/.." ABSOLUTE)
endif()

# ---------------------------------------------------------------------------
# The example executables.
# ---------------------------------------------------------------------------
set(PHYSICS_EXAMPLES
    01_dam_break
    02_faucet
    03_solver_interface
)

foreach(_ex IN LISTS PHYSICS_EXAMPLES)
    add_executable(sph_example_${_ex} ${_ex}.cpp)
    target_include_directories(sph_example_${_ex} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"   # sph_examples_common.h
        "${REPO_ROOT}"                  # CGLib/Math/...
        "${PHYSICS_ROOT}/Physics"       # WCSPHFluid.h, ISPHSolver.h, ...
    )
    target_link_libraries(sph_example_${_ex} PRIVATE PhysicsCore)
    set_target_properties(sph_example_${_ex} PROPERTIES CXX_STANDARD 17)
endforeach()
