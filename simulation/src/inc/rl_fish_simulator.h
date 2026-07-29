#pragma once

#include <Stonefish/core/SimulationManager.h>

class RLFishSimulator final : public sf::SimulationManager
{
public:
    explicit RLFishSimulator(sf::Scalar stepsPerSecond);

    void BuildScenario() override;

    // 后续接入强化学习时再加入：
    //
    // void SimulationStepCompleted(sf::Scalar timeStep) override;
    // void ResetEpisode();
    // void SetAction(...);
    // Observation GetObservation();
};

