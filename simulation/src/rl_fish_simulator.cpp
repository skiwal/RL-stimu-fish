#include "inc/rl_fish_simulator.h"

#include <core/ScenarioParser.h>
#include <core/SimulationApp.h>

RLFishSimulator::RLFishSimulator(sf::Scalar stepsPerSecond)
    : sf::SimulationManager(stepsPerSecond)
{
}

void RLFishSimulator::BuildScenario()
{
    sf::SimulationApp* app = sf::SimulationApp::getApp();

    if (app == nullptr)
    {
        throw std::runtime_error(
            "Stonefish SimulationApp is not initialized."
        );
    }

    sf::ScenarioParser parser(this);

    const std::string scenarioPath = 
    app->getDataPath() + "env/static_pool.scn";

    if (!parser.Parse(scenarioPath))
    {
        throw std::runtime_error(
            "Failed to load scenario: " + scenarioPath
        );
    }
}

