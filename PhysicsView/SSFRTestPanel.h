#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/Button.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/ComboBox.h"

namespace Phantom {
    class SSFluidRenderer;

class SSFRTestPanel : public ::VKG::IVkUIPanel {
public:
    enum class Preset { Sphere = 0, DamBreak = 1, Wave = 2 };

    void setVisible(bool v) { show_ = v; }
    bool isVisible()  const { return show_; }
    bool isActive()   const { return active_; }

    void bindSSFRRenderer(SSFluidRenderer* r) { ssfrRenderer_ = r; }

    bool consumeDirty();

    const std::vector<glm::vec3>& getPositions() const { return positions_; }

    void onImGui() override;

private:
    bool   show_   = false;
    bool   active_ = false;
    bool   dirty_  = false;
    Preset preset_ = Preset::Sphere;
    int    count_  = 3000;
    float  radius_ = 12.f;

    SSFluidRenderer* ssfrRenderer_ = nullptr;

    std::vector<glm::vec3> positions_;

    // Test generation
    Phantom::UI::BoolView  activeView_       { "Test Mode"         };
    Phantom::UI::ComboBox  presetCombo_      { "Preset"            };
    Phantom::UI::Button    generateButton_   { "Generate"          };

    // Debug panel
    Phantom::UI::BoolView  anisoView_        { "Anisotropic ON"    };
    Phantom::UI::BoolView  depthSmoothView_  { "Depth Smoothing ON"};
    Phantom::UI::Button    rawDepthButton_   { "Raw Depth"         };
    Phantom::UI::Button    smoothDepthButton_{ "Smooth Depth"      };
    Phantom::UI::Button    rawThickButton_   { "Raw Thick"         };
    Phantom::UI::Button    smoothThickButton_{ "Smooth Thick"      };
    Phantom::UI::Button    reflectionButton_ { "Reflection"        };
    Phantom::UI::Button    fullSSFRButton_   { "Full SSFR"         };

    bool widgetsInitialized_ = false;
    void initWidgets();

    void generate();
    void genSphere();
    void genDamBreak();
    void genWave();
    void onImGuiComparison();
};

} // namespace Phantom
