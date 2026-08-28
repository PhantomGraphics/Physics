#include "pch.h"

#include "../Physics/MVCFluid.h"
#include "../Physics/MVCParticle.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

// ---- Particle addition ---------------------------------------------------

TEST(MVCFluidTest, AddParticleIncreasesCount)
{
  MVCFluid fluid;
  EXPECT_EQ(fluid.getNumParticles(), 0);

  MVCParticle p;
  p.position = Vector3df(0.0f, 0.0f, 0.0f);
  fluid.addParticle(p);
  EXPECT_EQ(fluid.getNumParticles(), 1);
}

// ---- Static flag round-trip -----------------------------------------------

TEST(MVCFluidTest, SetStaticRoundTrips)
{
  MVCFluid fluid;
  EXPECT_FALSE(fluid.isStatic());

  fluid.setStatic(true);
  EXPECT_TRUE(fluid.isStatic());
}

// ---- Density getter/setter -------------------------------------------------

TEST(MVCFluidTest, DensityGetterReturnsSetValue)
{
  MVCFluid fluid;
  fluid.setDensity(800.0f);
  EXPECT_FLOAT_EQ(fluid.getDensity(), 800.0f);
}

// ---- Emitter (docs/todo/PLAN_physics_fluid_emitter.md) -------------------

TEST(MVCFluidTest, UpdateEmittersIsNoOpWithoutRegisteredEmitters)
{
  MVCFluid fluid;
  fluid.setDensity(1000.0f);

  fluid.updateEmitters(0.1f);

  EXPECT_EQ(fluid.getNumParticles(), 0);
}

TEST(MVCFluidTest, UpdateEmittersEmitsAtTheConfiguredRate)
{
  MVCFluid fluid;
  fluid.setDensity(1000.0f);

  Emitter e;
  e.rate = 100.0f;
  fluid.addEmitter(e);

  for (int step = 0; step < 10; ++step)
  {
    fluid.updateEmitters(0.01f);
  }

  EXPECT_EQ(fluid.getNumParticles(), 10);
}

TEST(MVCFluidTest, UpdateEmittersStopsAtMaxParticles)
{
  MVCFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setMaxParticles(5);

  Emitter e;
  e.rate = 10000.0f;
  fluid.addEmitter(e);

  fluid.updateEmitters(1.0f);

  EXPECT_EQ(fluid.getNumParticles(), 5);
}

TEST(MVCFluidTest, UpdateEmittersGivesSpawnedParticlesVelocityAlongDirectionAndDensity)
{
  MVCFluid fluid;
  fluid.setDensity(1234.0f);

  Emitter e;
  e.rate = 100.0f;
  e.direction = Vector3df(0.0f, 1.0f, 0.0f);
  e.speed = 2.0f;
  e.speedJitter = 0.0f;
  fluid.addEmitter(e);

  fluid.updateEmitters(0.01f);

  ASSERT_EQ(fluid.getNumParticles(), 1);
  const Vector3df vel = fluid.getParticles().velocities[0];
  EXPECT_NEAR(vel.x, 0.0f, 1.0e-5f);
  EXPECT_NEAR(vel.y, 2.0f, 1.0e-5f);
  EXPECT_NEAR(vel.z, 0.0f, 1.0e-5f);
  EXPECT_FLOAT_EQ(fluid.getParticles().densities[0], 1234.0f);
}

TEST(MVCFluidTest, ClearEmittersRemovesRegisteredEmitters)
{
  MVCFluid fluid;
  fluid.setDensity(1000.0f);

  Emitter e;
  fluid.addEmitter(e);
  EXPECT_EQ(fluid.getEmitters().size(), 1U);

  fluid.clearEmitters();
  EXPECT_TRUE(fluid.getEmitters().empty());

  fluid.updateEmitters(1.0f);
  EXPECT_EQ(fluid.getNumParticles(), 0);
}
