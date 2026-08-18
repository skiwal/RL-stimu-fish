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
    if (path.empty())
    {
        return "../data/";
    }


    if (path.back() != '/')
    {
        path.push_back('/');
    }


    return path;
}


PoolTestMode ParseTestMode(
    const std::string& name)
{
    if (name == "neutral")
    {
        return PoolTestMode::Neutral;
    }


    if (name == "straight")
    {
        return PoolTestMode::Straight;
    }


    if (name == "turn_left")
    {
        return PoolTestMode::TurnLeft;
    }


    if (name == "turn_right")
    {
        return PoolTestMode::TurnRight;
    }


    if (name == "dive")
    {
        return PoolTestMode::Dive;
    }


    if (name == "rise")
    {
        return PoolTestMode::Rise;
    }


    if (name == "roll_left")
    {
        return PoolTestMode::RollLeft;
    }


    if (name == "roll_right")
    {
        return PoolTestMode::RollRight;
    }


    if (name == "external")
    {
        return PoolTestMode::External;
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
        << " <data-dir> <test-mode>\n\n"

        << "Example:\n\n"
        << "  "
        << executable
        << " ./data/ straight\n\n"

        << "Available modes:\n"
        << "  neutral\n"
        << "  straight\n"
        << "  turn_left\n"
        << "  turn_right\n"
        << "  dive\n"
        << "  rise\n"
        << "  roll_left\n"
        << "  roll_right\n"
        << "  external\n\n";
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
                : "straight";


        const PoolTestMode testMode =
            ParseTestMode(
                testModeName);


        // ========================================================
        // Rendering
        // ========================================================

        sf::RenderSettings
            renderSettings;


        /*
            使用你本机 Stonefish API：

                windowW
                windowH

            而不是：
                windowWidth
                windowHeight
        */

        renderSettings.windowW =
            1920;


        renderSettings.windowH =
            1080;


        sf::HelperSettings
            helperSettings;


        // ========================================================
        // Simulation
        // ========================================================

        StaticPoolSimulator simulator(
            2000.0,
            testMode);


        std::cout
            << "Starting Bionic Fish V1 test: "
            << PoolTestModeName(
                   testMode)
            << '\n';


        sf::GraphicalSimulationApp app(
            "Bionic Fish V1 - Static Pool",
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
