#include <Stonefish/core/GraphicalSimulationApp.h>

#include "inc/tendon_stage_r2_simulator.h"

#include <iostream>
#include <string>

namespace {

std::string NormalizeDataPath(std::string path)
{
    if (path.empty())
        path = "../data/";

    if (path.back() != '/')
        path.push_back('/');

    return path;
}

}

int main(int argc, char** argv)
{
    const std::string dataPath =
        NormalizeDataPath(
            argc > 1
                ? argv[1]
                : "../data/");

    sf::RenderSettings renderSettings;
    renderSettings.windowW = 1920;
    renderSettings.windowH = 1080;

    sf::HelperSettings helperSettings;

    constexpr sf::Scalar physicsSPS = 2000.0;

    TendonStageR2Simulator simulator(
        physicsSPS);

    std::cout
        << "\nStarting Caudal Spring Thrust Test\n"
        << "  body         = fixed\n"
        << "  motor        = disabled\n"
        << "  tendon       = direct tension\n"
        << "  peak force   = 3 N\n"
        << "  frequency    = 0.60 Hz\n"
        << "  caudal       = revolute + spring + damping\n"
        << "  spring k     = 0.05 Nm/rad\n"
        << "  damping c    = 0.005 Nms/rad\n"
        << "  caudal limit = +/-20 deg\n"
        << "  forward      = +X\n"
        << "  physics      = 2000 Hz\n"
        << std::endl;

    sf::GraphicalSimulationApp app(
        "BionicFish - Caudal Spring Thrust Test",
        dataPath,
        renderSettings,
        helperSettings,
        &simulator);

    app.Run();

    return 0;
}
