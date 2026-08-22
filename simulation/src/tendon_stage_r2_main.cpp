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

    render.windowW =
        1920;

    render.windowH =
        1080;


    sf::HelperSettings helpers;


    // Stiff tendon:
    // k = 20000 N/m.
    //
    // Use a higher physics rate than the previous
    // constant-force diagnostic.
    constexpr sf::Scalar physicsSPS =
        4000.0;


    TendonStageR2Simulator simulator(
        physicsSPS);


    std::cout
        << "\nStarting Stage 3 Motor + Dual Elastic Tendon\n"
        << "  motor           = real M1Joint\n"
        << "  motor mode      = velocity\n"
        << "  motor frequency = 1.25 Hz\n"
        << "  tendon k        = 20000 N/m\n"
        << "  tendon c        = 10 Ns/m\n"
        << "  routing         = [0,0,0,1,1]\n"
        << "  tail k          = 0.65 Nm/rad\n"
        << "  caudal          = fixed\n"
        << "  physics         = 4000 Hz\n"
        << "  output          = dual_tendon_motor.csv\n"
        << std::endl;


    sf::GraphicalSimulationApp app(
        "BionicFish - Motor + Dual Elastic Tendon",
        dataPath,
        render,
        helpers,
        &simulator);


    app.Run();

    return 0;
}
