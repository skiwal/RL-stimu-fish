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
        NormalizeDataPath(argc > 1 ? argv[1] : "../data/");

    sf::RenderSettings render;
    render.windowW = 1920;
    render.windowH = 1080;

    sf::HelperSettings helper;

    constexpr sf::Scalar physicsHz = 2000.0;
    TendonStageR2Simulator simulator(physicsHz);

    std::cout
        << "\nStarting Tethered Thrust Test\n"
        << "  body         = fixed\n"
        << "  motor        = disabled\n"
        << "  tendon       = continuous direct tension\n"
        << "  peak force   = 3 N\n"
        << "  frequency    = 1.25 Hz\n"
        << "  forward      = +X\n"
        << "  physics      = 2000 Hz\n\n";

    sf::GraphicalSimulationApp app(
        "BionicFish - Tethered Thrust Test",
        dataPath,
        render,
        helper,
        &simulator);

    app.Run();
    return 0;
}
