#include <gtest/gtest.h>
#include "../PhysicsView/FlameSmokeShadow.h"

using Phantom::FlameSmokeShadow;

TEST(FlameSmokeShadow, RayChordAndInsideOrigin) {
    const FlameSmokeShadow::Puff puff{{0, 2, 0}, 2, 3};
    EXPECT_NEAR(FlameSmokeShadow::rayDepth(puff, {0, 0, 0}, {0, 1, 0}), 3.0f, 1.0e-6f);
    EXPECT_NEAR(FlameSmokeShadow::rayDepth(puff, {0, 2, 0}, {0, 1, 0}), 1.5f, 1.0e-6f);
    EXPECT_FLOAT_EQ(FlameSmokeShadow::rayDepth(puff, {0, 0, 0}, {0, -1, 0}), 0.0f);
    EXPECT_FLOAT_EQ(FlameSmokeShadow::rayDepth(puff, {2, 0, 0}, {0, 1, 0}), 0.0f);
    EXPECT_FLOAT_EQ(FlameSmokeShadow::rayDepth({{0, 2, 0}, 0, 3}, {0, 0, 0}, {0, 1, 0}), 0.0f);
}

TEST(FlameSmokeShadow, ExcludesReceiverAndOnlyAttenuatesTowardLight) {
    FlameSmokeShadow shadow;
    shadow.build({{{0, 0, 0}, 2, 5}, {{0, 3, 0}, 2, 2}, {{0, -3, 0}, 2, 4}});
    EXPECT_NEAR(shadow.opticalDepth(0, {0, 1, 0}), 2.0f, 1.0e-6f);
    EXPECT_NEAR(shadow.opticalDepth(0, {0, -1, 0}), 4.0f, 1.0e-6f);
    shadow.build({{{0, 0, 0}, 2, 5}});
    EXPECT_FLOAT_EQ(shadow.opticalDepth(0, {0, 1, 0}), 0.0f);
    shadow.build({});
    EXPECT_FLOAT_EQ(shadow.opticalDepth(0, {0, 1, 0}), 0.0f);
}

TEST(FlameSmokeShadow, BvhMatchesDirectSumForRotatedLightAndOverlappingPuffs) {
    std::vector<FlameSmokeShadow::Puff> puffs;
    for (int i = 0; i < 80; ++i)
        puffs.push_back({{float(i % 5) * 0.7f, float(i / 5) * 0.3f, float(i % 3) * 0.4f}, 1.2f, float(i % 7) * 0.1f});
    FlameSmokeShadow shadow;
    shadow.build(puffs);
    for (const glm::vec3 direction : {glm::vec3(0, 1, 0), glm::normalize(glm::vec3(1, 2, -0.3f))}) {
        for (unsigned i = 0; i < puffs.size(); ++i) {
            float expected = 0;
            for (unsigned j = 0; j < puffs.size(); ++j)
                if (i != j) expected += FlameSmokeShadow::rayDepth(puffs[j], puffs[i].centre, direction);
            EXPECT_NEAR(shadow.opticalDepth(i, direction), expected, 1.0e-5f);
        }
    }
}
