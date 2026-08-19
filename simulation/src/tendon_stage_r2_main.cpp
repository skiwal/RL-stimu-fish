#include <Stonefish/core/GraphicalSimulationApp.h>

#include "inc/tendon_stage_r2_simulator.h"

#include <iostream>
#include <string>

namespace
{

    std::string
    NormalizeDataPath(
        std::string path)
    {
        if (
            path.empty())
        {
            path =
                "../data/";
        }

        if (
            path.back() != '/')
        {
            path.push_back(
                '/');
        }

        return path;
    }

}

int main(
    int argc,
    char **argv)
{
    const std::string
        dataPath =
            NormalizeDataPath(
                argc > 1
                    ? argv[1]
                    : "../data/");

    sf::RenderSettings
        renderSettings;

    renderSettings.windowW =
        1920;

    renderSettings.windowH =
        1080;

    sf::HelperSettings
        helperSettings;

    constexpr sf::Scalar
        physicsStepsPerSecond =
            2000.0;

    TendonStageR2Simulator
        simulator(
            physicsStepsPerSecond);

    std::cout
        << "\nStarting Stage R2-A\n"
        << "  M1 model     = kinematic crank shaft\n"
        << "  tendons      = generalized spatial tendons\n"
        << "  body fixed   = true\n"
        << "  physics      = 2000 Hz\n"
        << std::endl;

    sf::GraphicalSimulationApp
        app(
            "BionicFish - Stage R2-A Tendon Drive",
            dataPath,
            renderSettings,
            helperSettings,
            &simulator);

    app.Run();

    return 0;
}
