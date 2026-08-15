#pragma once

#include <Stonefish/core/SimulationManager.h>

#include "river_chunk_manager.h"

namespace sf
{
class SolidEntity;
}

class RLFishSimulator final
    : public sf::SimulationManager
{
public:
    explicit RLFishSimulator(
        sf::Scalar stepsPerSecond);

    void BuildScenario() override;

    void SimulationStepCompleted(
        sf::Scalar timeStep) override;

private:
    sf::SolidEntity* fish_ = nullptr;

    RiverChunkManager river_;
};
