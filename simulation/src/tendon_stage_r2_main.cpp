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

    sf::RenderSettings render;
    render.windowW = 1920;
    render.windowH = 1080;

    sf::HelperSettings helpers;

    constexpr sf::Scalar physicsSPS =
        2000.0;

    TendonStageR2Simulator simulator(
        physicsSPS);

    std::cout
        << "\nStarting PHASE 0 Thrust Validation\n"
        << "  body             = fixed\n"
        << "  motor            = disabled\n"
        << "  tendon peak      = 3.0 N\n"
        << "  frequency        = 0.60 Hz\n"
        << "  caudal k         = 0.50 Nm/rad\n"
        << "  caudal damping   = 0.005 Nms/rad\n"
        << "  anchor reaction  = enabled\n"
        << "  jacobian check   = enabled\n"
        << "  loadcell         = reconstructed support reaction\n"
        << "  forward          = +X\n"
        << "  physics          = 2000 Hz\n"
        << std::endl;

    sf::GraphicalSimulationApp app(
        "BionicFish - Phase 0 Thrust Validation",
        dataPath,
        render,
        helpers,
        &simulator);

    app.Run();

    return 0;
}
