#include <Stonefish/core/GraphicalSimulationApp.h>

#include "inc/static_pool_simulator.h"

#include <iostream>
#include <stdexcept>
#include <string>


namespace
{

std::string NormalizeDataDirectory(
    std::string path)
{
    if (
        path.empty())
    {
        return "../data/";
    }


    if (
        path.back()
        != '/')
    {
        path.push_back(
            '/');
    }


    return path;
}


PoolTestMode ParseTestMode(
    const std::string& name)
{
    if (
        name == "neutral")
    {
        return
            PoolTestMode::Neutral;
    }


    if (
        name == "straight")
    {
        return
            PoolTestMode::Straight;
    }


    if (
        name == "turn_left")
    {
        return
            PoolTestMode::TurnLeft;
    }


    if (
        name == "turn_right")
    {
        return
            PoolTestMode::TurnRight;
    }


    if (
        name == "dive")
    {
        return
            PoolTestMode::Dive;
    }


    if (
        name == "rise")
    {
        return
            PoolTestMode::Rise;
    }


    if (
        name == "roll_left")
    {
        return
            PoolTestMode::RollLeft;
    }


    if (
        name == "roll_right")
    {
        return
            PoolTestMode::RollRight;
    }


    if (
        name == "external")
    {
        return
            PoolTestMode::External;
    }


    throw std::invalid_argument(
        "Unknown pool test mode: "
        + name);
}


void PrintUsage(
    const char* executable)
{
    std::cout
        << "\nUsage:\n\n"
        << "  "
        << executable
        << " <data-dir> neutral\n\n"

        << "Stage R1-A is a passive free-decay experiment.\n"
        << "Use:\n\n"

        << "  "
        << executable
        << " ./data/ neutral\n\n";
}

} // namespace


int main(
    int argc,
    char** argv)
{
    try
    {
        const std::string dataDirectory =
            NormalizeDataDirectory(
                argc > 1
                    ? argv[1]
                    : "../data/");


        const std::string testModeName =
            argc > 2
                ? argv[2]
                : "neutral";


        const PoolTestMode testMode =
            ParseTestMode(
                testModeName);


        sf::RenderSettings
            renderSettings;


        renderSettings.windowW =
            1920;


        renderSettings.windowH =
            1080;


        sf::HelperSettings
            helperSettings;


        // ========================================================
        // Stage R1
        //
        // Keep 2000 Hz because the previous stiff-tail tests
        // demonstrated that 500 Hz could excite nonphysical
        // high-frequency modes.
        // ========================================================

        constexpr sf::Scalar
            physicsStepsPerSecond =
            2000.0;


        StaticPoolSimulator simulator(
            physicsStepsPerSecond,
            testMode);


        std::cout
            << "\n"
            << "Starting BionicFish Stage R1-A\n"
            << "  experiment  = five-passive-joint free decay\n"
            << "  M1          = absent / disabled\n"
            << "  tendons     = absent\n"
            << "  body        = fixed\n"
            << "  physics SPS = "
            << physicsStepsPerSecond
            << "\n"
            << std::endl;


        sf::GraphicalSimulationApp app(
            "BionicFish - Stage R1-A Five Passive Joints",
            dataDirectory,
            renderSettings,
            helperSettings,
            &simulator);


        app.Run();


        return 0;
    }
    catch (
        const std::exception& exception)
    {
        std::cerr
            << "\nERROR: "
            << exception.what()
            << "\n";


        PrintUsage(
            argv[0]);


        return 1;
    }
}
