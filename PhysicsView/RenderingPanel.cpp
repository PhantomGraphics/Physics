#include "pch.h"
#include "RenderingPanel.h"

#include "RenderBackground.h"
#include "GltfBodyRenderer.h"

namespace Phantom {

// Math::Vector3df is glm::vec<3,float> (CGLib/Math/Vector3d.h), so
// Vector3dView values interoperate with glm::vec3 directly.

void RenderingPanel::init() { buildUi(); }

std::string RenderingPanel::backgroundStatusText() const {
    if (!bg_) return {};
    std::string s = "primitives: " + std::to_string(bg_->primitiveCount());
    if (!bg_->backgroundPath().empty()) s += "  |  " + bg_->backgroundPath();
    return s;
}

std::string RenderingPanel::environmentStatusText() const {
    if (!bg_) return {};
    return bg_->hasEnvironment() ? ("env: " + bg_->environmentDir())
                                 : std::string("env: (default)");
}

std::string RenderingPanel::rigidStatusText() const {
    if (!rigidBody_) return {};
    return "shaded instances: " + std::to_string(rigidBody_->instanceCount());
}

void RenderingPanel::buildUi() {
    if (uiBuilt_) return;
    uiBuilt_ = true;

    loadBtn_.setFunction([this] {
        if (bg_) bg_->loadBackground(bgPathInput_.getValue());
    });
    clearBtn_.setFunction([this] {
        if (bg_) bg_->clearBackground();
    });

    // pos/rot/scale are pushed together in drawContents(); the slider still needs
    // a getter so it shows the live value.
    xfScale_.bind([this] { return bg_ ? bg_->transformScale() : 1.f; },
                  [](float) {});

    setEnvBtn_.setFunction([this] {
        if (bg_) bg_->setEnvironment(envDirInput_.getValue());
    });
    clearEnvBtn_.setFunction([this] {
        if (bg_) bg_->clearEnvironment();
    });
    useIblCheck_.bind([this] { return bg_ && bg_->useIBL(); },
                      [this](bool v) { if (bg_) bg_->setUseIBL(v); });

    lightInt_.bind([this] { return bg_ ? bg_->lightIntensity() : 3.f; },
                   [](float) {}); // pushed with dir/color in drawContents()

    rigidModeCombo_.addItem("Wireframe");
    rigidModeCombo_.addItem("Shaded");
    rigidModeCombo_.addItem("Both");
    rigidModeCombo_.bind(
        [this] { return rigidBody_ ? static_cast<int>(rigidBody_->mode()) : 0; },
        [this](int v) {
            if (rigidBody_) rigidBody_->setMode(static_cast<GltfBodyRenderer::Mode>(v));
        });
    rigidModeCombo_.setVisibleWhen([this] { return rigidBody_ != nullptr; });
    rigidStatus_.setVisibleWhen([this] { return rigidBody_ != nullptr; });

    contents_.add(&bgHeader_);
    contents_.add(&bgPathInput_);
    contents_.add(&loadBtn_);
    contents_.add(&clearBtn_);
    contents_.add(&bgStatus_);
    contents_.add(&sep1_);
    contents_.add(&xfHeader_);
    contents_.add(&xfPos_);
    contents_.add(&xfRot_);
    contents_.add(&xfScale_);
    contents_.add(&sep2_);
    contents_.add(&envHeader_);
    contents_.add(&envDirInput_);
    contents_.add(&setEnvBtn_);
    contents_.add(&clearEnvBtn_);
    contents_.add(&useIblCheck_);
    contents_.add(&envStatus_);
    contents_.add(&sep3_);
    contents_.add(&lightHeader_);
    contents_.add(&lightDir_);
    contents_.add(&lightColor_);
    contents_.add(&lightInt_);
    contents_.add(&sep4_);
    contents_.add(&rigidHeader_);
    contents_.add(&rigidModeCombo_);
    contents_.add(&rigidStatus_);
}

void RenderingPanel::drawContents() {
    if (!uiBuilt_) buildUi();

    if (bg_ && !seeded_) {
        seeded_ = true;
        bgPathInput_.setValue(bg_->backgroundPath());
        envDirInput_.setValue(bg_->environmentDir());
        xfPos_.setValue(bg_->transformPosition());
        xfRot_.setValue(bg_->transformRotationDeg());
        xfScale_.setValue(bg_->transformScale());
        lightDir_.setValue(bg_->lightDirection());
        lightColor_.setValue(bg_->lightColor());
        lightInt_.setValue(bg_->lightIntensity());
    }

    contents_.show();

    // Push the value-editable knobs every frame (cheap stores, no Vulkan work).
    if (bg_) {
        bg_->setTransform(xfPos_.getValue(), xfRot_.getValue(), xfScale_.getValue());
        bg_->setLight(lightDir_.getValue(), lightColor_.getValue(), lightInt_.getValue());
    }
}

} // namespace Phantom
