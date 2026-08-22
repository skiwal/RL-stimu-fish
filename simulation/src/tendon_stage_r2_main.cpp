#include <Stonefish/core/GraphicalSimulationApp.h>

#include "inc/tendon_stage_r2_simulator.h"

#include <iostream>
#include <string>

namespace
{

std::string NormalizeDataPath(
    std::string path)
{
    if (path.empty())
        path = "../data/";

    if (path.back() != '/')
        path.push_back('/');

    return path;
}

}


int main(
    int argc,
    char** argv)
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
        << "\nStarting S-Bend Tendon Diagnostic\n"
        << "  body          = fixed\n"
        << "  caudal        = fixed to Tail4\n"
        << "  left tendon   = constant 1.0 N\n"
        << "  right tendon  = 0.0 N\n"
        << "  routing       = [0,0,0,1,1]\n"
        << "  tail k        = 0.65 Nm/rad\n"
        << "  tail c        = 0.0 Nms/rad\n"
        << "  motor/crank   = bypassed\n"
        << "  physics       = 2000 Hz\n"
        << "  output        = s_bend_diagnostic.csv\n"
        << std::endl;

    sf::GraphicalSimulationApp app(
        "BionicFish - S-Bend Tendon Diagnostic",
        dataPath,
        render,
        helpers,
        &simulator);

    app.Run();

    return 0;
}
