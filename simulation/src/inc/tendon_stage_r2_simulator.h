#pragma once

#include <Stonefish/core/SimulationManager.h>
#include "tendon_tail_actuator.h"

#include <array>
#include <fstream>

namespace sf {
class Robot;
class FeatherstoneRobot;
class FeatherstoneEntity;
class SolidEntity;
}

class TendonStageR2Simulator final : public sf::SimulationManager
{
public:
    explicit TendonStageR2Simulator(sf::Scalar stepsPerSecond);

    void BuildScenario() override;
    void SimulationStepCompleted(sf::Scalar timeStep) override;

private:
    void BindFish();
    int FindJointIndex(const char* name) const;
    int FindLinkIndex(const char* name) const;

    void SetNeutralInitialCondition();
    void RegisterTendonActuator();
    void ConfigureCamera();

    sf::Vector3 GetHydroForce(sf::SolidEntity* solid) const;
    sf::Vector3 GetPropulsorForce() const;

    void OpenCsv();
    void RecordSample();
    void PrintTelemetry();

private:
    sf::Robot* fishRobot_ = nullptr;
    sf::FeatherstoneRobot* fishFeatherstoneRobot_ = nullptr;
    sf::FeatherstoneEntity* fishDynamics_ = nullptr;
    sf::SolidEntity* fishBody_ = nullptr;

    int bodyLinkIndex_ = -1;
    int caudalLinkIndex_ = -1;

    std::array<int, 5> tailJointIndices_{{-1,-1,-1,-1,-1}};
    std::array<int, 5> tailLinkIndices_{{-1,-1,-1,-1,-1}};

    std::array<sf::SolidEntity*, 5> tailSolids_{{nullptr,nullptr,nullptr,nullptr,nullptr}};
    sf::SolidEntity* caudalFin_ = nullptr;

    TendonTailActuator* tendonActuator_ = nullptr;

    std::ofstream csv_;

    sf::Scalar elapsedTimeS_ = 0.0;
    sf::Scalar csvPeriodS_ = 0.002;       // 500 Hz
    sf::Scalar consolePeriodS_ = 0.05;    // 20 Hz
    sf::Scalar lastCsvTimeS_ = -1.0;
    sf::Scalar lastConsoleTimeS_ = -1.0;

    // Ignore startup transient when calculating mean thrust.
    sf::Scalar meanStartTimeS_ = 5.0;
    sf::Scalar meanAccumTimeS_ = 0.0;
    sf::Scalar impulseFxNs_ = 0.0;
    sf::Scalar impulseFyNs_ = 0.0;
    sf::Scalar impulseFzNs_ = 0.0;
};
