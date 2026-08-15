#include <Stonefish/core/GraphicalSimulationApp.h>

#include "inc/rl_fish_simulator.h"

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

} // namespace

int main(
    int argc,
    char** argv)
{
    sf::RenderSettings renderSettings;

    renderSettings.windowW = 1920;
    renderSettings.windowH = 1080;

    sf::HelperSettings helperSettings;

    RLFishSimulator simulator(500.0);

    const std::string dataDirectory =
        NormalizeDataDirectory(
            argc > 1
                ? argv[1]
                : "../data/");

    sf::GraphicalSimulationApp app(
        "RL Bionic Fish - Basic River",
        dataDirectory,
        renderSettings,
        helperSettings,
        &simulator);

    app.Run();

    return 0;
}
