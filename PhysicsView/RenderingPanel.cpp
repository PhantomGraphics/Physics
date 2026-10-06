#include "pch.h"
#include "RenderingPanel.h"

#include "RenderBackground.h"
#include "GltfBodyRenderer.h"
#include "GltfSoftRenderer.h"

namespace Phantom {

// Math::Vector3df is glm::vec<3,float> (CGLib/Math/Vector3d.h), so
// Vector3dView values interoperate with glm::vec3 directly.

namespace {
std::string fmtF(float v) {
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.9g", v);
    return buf;
}
const char* modeName(BodyRenderMode m) {
    return m == BodyRenderMode::Shaded ? "shaded" : m == BodyRenderMode::Both ? "both" : "wire";
}
}

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

std::string RenderingPanel::softStatusText() const {
    if (!softBody_) return {};
    return "shaded instances: " + std::to_string(softBody_->instanceCount());
}

void RenderingPanel::buildUi() {
    if (uiBuilt_) return;
    uiBuilt_ = true;

    loadBtn_.setFunction([this] {
        if (sink_) emit("LoadRenderBackground:" + bgPathInput_.getValue());
        else if (bg_) bg_->loadBackground(bgPathInput_.getValue());
    });
    clearBtn_.setFunction([this] {
        if (sink_) emit("ClearRenderBackground");
        else if (bg_) bg_->clearBackground();
    });

    // pos/rot/scale are pushed together in drawContents(); the slider still needs
    // a getter so it shows the live value.
    xfScale_.bind([this] { return bg_ ? bg_->transformScale() : 1.f; },
                  [](float) {});

    setEnvBtn_.setFunction([this] {
        if (sink_) emit("SetEnvironment:" + envDirInput_.getValue());
        else if (bg_) bg_->setEnvironment(envDirInput_.getValue());
    });
    clearEnvBtn_.setFunction([this] {
        if (sink_) emit("ClearRenderEnvironment");
        else if (bg_) bg_->clearEnvironment();
    });
    useIblCheck_.bind([this] { return bg_ && bg_->useIBL(); },
                      [this](bool v) {
                          if (sink_) emit(std::string("SetRenderUseIBL:") + (v ? "1" : "0"));
                          else if (bg_) bg_->setUseIBL(v);
                      });
    shadowCheck_.bind([this] { return bg_ && bg_->castShadows(); },
                      [this](bool v) {
                          if (sink_) emit(std::string("SetShadowEnabled:") + (v ? "1" : "0"));
                          else if (bg_) bg_->setCastShadows(v);
                      });

    lightInt_.bind([this] { return bg_ ? bg_->lightIntensity() : 3.f; },
                   [](float) {}); // pushed with dir/color in drawContents()

    for (auto* c : { &rigidModeCombo_, &softModeCombo_ }) {
        c->addItem("Wireframe");
        c->addItem("Shaded");
        c->addItem("Both");
    }
    rigidModeCombo_.bind(
        [this] { return rigidBody_ ? static_cast<int>(rigidBody_->mode()) : 0; },
        [this](int v) {
            if (sink_) emit(std::string("SetRigidRenderMode:") + modeName(static_cast<BodyRenderMode>(v)));
            else if (rigidBody_) rigidBody_->setMode(static_cast<BodyRenderMode>(v));
        });
    rigidModeCombo_.setVisibleWhen([this] { return rigidBody_ != nullptr; });
    rigidStatus_.setVisibleWhen([this] { return rigidBody_ != nullptr; });
    softModeCombo_.bind(
        [this] { return softBody_ ? static_cast<int>(softBody_->mode()) : 0; },
        [this](int v) {
            if (sink_) emit(std::string("SetSoftRenderMode:") + modeName(static_cast<BodyRenderMode>(v)));
            else if (softBody_) softBody_->setMode(static_cast<BodyRenderMode>(v));
        });
    softModeCombo_.setVisibleWhen([this] { return softBody_ != nullptr; });
    softStatus_.setVisibleWhen([this] { return softBody_ != nullptr; });

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
    contents_.add(&shadowCheck_);
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
    contents_.add(&softModeCombo_);
    contents_.add(&softStatus_);
}

void RenderingPanel::drawContents() {
    if (!uiBuilt_) buildUi();

    if (bg_ && !seeded_) {
        seeded_ = true;
        bgPathInput_.setValue(bg_->backgroundPath());
        envDirInput_.setValue(bg_->environmentDir());
    }
    // Transform / light: the render state is the source of truth. Seed the widgets from it
    // each frame, then turn an edit (a widget that no longer equals the state) into a command.
    if (bg_) {
        xfPos_.setValue(bg_->transformPosition());
        xfRot_.setValue(bg_->transformRotationDeg());
        xfScale_.setValue(bg_->transformScale());
        lightDir_.setValue(bg_->lightDirection());
        lightColor_.setValue(bg_->lightColor());
        lightInt_.setValue(bg_->lightIntensity());
    }

    contents_.show();

    if (bg_) {
        const auto pos = xfPos_.getValue();
        const auto rot = xfRot_.getValue();
        const float sc = xfScale_.getValue();
        const auto dir = lightDir_.getValue();
        const auto col = lightColor_.getValue();
        const float inten = lightInt_.getValue();
        if (!sink_) {
            bg_->setTransform(pos, rot, sc);
            bg_->setLight(dir, col, inten);
        } else {
            if (pos != bg_->transformPosition() || rot != bg_->transformRotationDeg() || sc != bg_->transformScale())
                emit("SetRenderBackgroundTransform:" + fmtF(pos.x) + "," + fmtF(pos.y) + "," + fmtF(pos.z) + "," +
                     fmtF(rot.x) + "," + fmtF(rot.y) + "," + fmtF(rot.z) + "," + fmtF(sc));
            if (dir != bg_->lightDirection() || col != bg_->lightColor() || inten != bg_->lightIntensity())
                emit("SetLight:" + fmtF(dir.x) + "," + fmtF(dir.y) + "," + fmtF(dir.z) + "," +
                     fmtF(col.x) + "," + fmtF(col.y) + "," + fmtF(col.z) + "," + fmtF(inten));
        }
    }
}

} // namespace Phantom
