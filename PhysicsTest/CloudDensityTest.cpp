#include "pch.h"

#include "../Physics/CloudDensity.h"

#include <cmath>

using namespace Phantom::Physics;
using Phantom::Math::Vector3dd;
using Phantom::Math::Vector3df;

TEST(CloudDensityTest, GridCoversDomainWithCubicCells)
{
  const auto d = CloudDensity::makeGridDesc(Vector3dd(0.0), Vector3dd(500.0, 500.0, 1000.0), 64);
  EXPECT_NEAR(d.cellSize, 1000.0f / 64.0f, 1e-4f);
  EXPECT_EQ(d.nz, 64u);
  EXPECT_EQ(d.nx, 32u);
  EXPECT_EQ(d.ny, 32u);
  EXPECT_GE(d.nx * d.cellSize, 500.0f - 1e-3f);
}

TEST(CloudDensityTest, SingleParticleConservesMass)
{
  CloudParticleSoA soa;
  soa.add(Vector3dd(500.0, 500.0, 500.0), Vector3dd(0.0), 8000.0, 290.0, 0.0, 1.0e-3);   // 8 kg of cloud water
  const auto desc = CloudDensity::makeGridDesc(Vector3dd(0.0), Vector3dd(1000.0), 64);
  Phantom::Volume::ScalarGrid3D grid;
  const CloudDensityStats st = CloudDensity::reconstruct(soa, desc, 60.0, grid);
  EXPECT_NEAR(st.particleMass, 8.0, 1e-12);
  EXPECT_LT(st.relativeError, 0.01);
  EXPECT_EQ(st.cloudyParticles, 1u);
  EXPECT_GT(grid.sample(Vector3df(500.0f, 500.0f, 500.0f)), 0.0f);
  EXPECT_EQ(grid.sample(Vector3df(100.0f, 100.0f, 100.0f)), 0.0f);
}

TEST(CloudDensityTest, UniformLatticeGivesUniformInteriorDensity)
{
  CloudParticleSoA soa;
  const double dx = 62.5;
  const double m = 250000.0, qc = 4.0e-4;   // rho_cloud = m*qc/dx^3 = 0.4096 kg/m^3
  for (double z = dx / 2; z < 1000.0; z += dx)
    for (double y = dx / 2; y < 1000.0; y += dx)
      for (double x = dx / 2; x < 1000.0; x += dx) soa.add(Vector3dd(x, y, z), Vector3dd(0.0), m, 290.0, 0.0, qc);
  const auto desc = CloudDensity::makeGridDesc(Vector3dd(0.0), Vector3dd(1000.0), 64);
  Phantom::Volume::ScalarGrid3D grid;
  const CloudDensityStats st = CloudDensity::reconstruct(soa, desc, 1.5 * dx, grid);
  const double expected = m * qc / (dx * dx * dx);
  for (float p : { 400.0f, 500.0f, 613.0f }) {
    EXPECT_NEAR(grid.sample(Vector3df(p, 500.0f, 470.0f)), expected, 0.03 * expected);
  }
  // Mass is lost only where kernels are cut by the domain walls; reported, not hidden.
  EXPECT_LT(st.relativeError, 0.10);
}

TEST(CloudDensityTest, NoCloudWaterGivesEmptyGrid)
{
  CloudParticleSoA soa;
  soa.add(Vector3dd(10.0), Vector3dd(0.0), 1000.0, 290.0, 0.01, 0.0);
  const auto desc = CloudDensity::makeGridDesc(Vector3dd(0.0), Vector3dd(100.0), 16);
  Phantom::Volume::ScalarGrid3D grid;
  const CloudDensityStats st = CloudDensity::reconstruct(soa, desc, 10.0, grid);
  EXPECT_EQ(st.cloudyParticles, 0u);
  EXPECT_EQ(st.gridMass, 0.0);
  for (float v : grid.data()) EXPECT_EQ(v, 0.0f);
}

TEST(CloudDensityTest, ParticleOnWallReportsTruncatedMass)
{
  CloudParticleSoA soa;
  soa.add(Vector3dd(0.0, 500.0, 500.0), Vector3dd(0.0), 8000.0, 290.0, 0.0, 1.0e-3);
  const auto desc = CloudDensity::makeGridDesc(Vector3dd(0.0), Vector3dd(1000.0), 64);
  Phantom::Volume::ScalarGrid3D grid;
  const CloudDensityStats st = CloudDensity::reconstruct(soa, desc, 60.0, grid);
  EXPECT_GT(st.relativeError, 0.3);   // about half the kernel lies outside the grid
  EXPECT_LT(st.gridMass, st.particleMass);
}
