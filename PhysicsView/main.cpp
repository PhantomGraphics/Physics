#include "pch.h"
#include "FluidApp.h"

int main(int argc, char* argv[])
{
    std::string scenarioPath;
    bool        noExitOnComplete = false;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--run-scenario" && i + 1 < argc) {
            scenarioPath = argv[++i];
        } else if (arg == "--no-exit-on-complete") {
            noExitOnComplete = true;
        }
    }

    Phantom::FluidApp app(1280, 720, "Vulkan Fluid View");

    if (!scenarioPath.empty()) {
        // Command-line scenario runs are non-interactive and must not read or
        // write the GUI layout file. Scenario Browser runs remain interactive
        // and therefore keep layout persistence enabled.
        app.disableInteractiveLayoutPersistence();
        if (!app.loadScenario(scenarioPath)) {
            fprintf(stderr, "[Scenario] Failed to load: %s\n", scenarioPath.c_str());
            return 1;
        }
        app.setExitOnScenarioComplete(!noExitOnComplete);
    }

    app.run(argc, argv);
    return app.getExitCode();
}
