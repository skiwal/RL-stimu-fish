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
    void SimulationStepCompleted(sf::Scalar dt) override;

private:
    struct HydroForce {
        sf::Vector3 drag{0.0,0.0,0.0};
        sf::Vector3 reactive{0.0,0.0,0.0};
        sf::Vector3 total{0.0,0.0,0.0};
    };

    void BindFish();
    int FindJointIndex(const char* name) const;
    int FindLinkIndex(const char* name) const;
    void SetNeutralInitialCondition();
    void RegisterTendonActuator();
    void ConfigureCamera();

    HydroForce GetHydroForce(sf::SolidEntity* solid) const;
    HydroForce GetPropulsorForce() const;

    void OpenCsv();
    void RecordSample();
    void PrintTelemetry();

    sf::Robot* fishRobot_ = nullptr;
    sf::FeatherstoneRobot* fishFeatherstoneRobot_ = nullptr;
    sf::FeatherstoneEntity* fishDynamics_ = nullptr;
    sf::SolidEntity* fishBody_ = nullptr;

    int bodyLinkIndex_ = -1;
    int caudalLinkIndex_ = -1;

    std::array<int,5> tailJointIndices_{{-1,-1,-1,-1,-1}};
    std::array<int,5> tailLinkIndices_{{-1,-1,-1,-1,-1}};
    std::array<sf::SolidEntity*,5> tailSolids_{{nullptr,nullptr,nullptr,nullptr,nullptr}};
    sf::SolidEntity* caudalFin_ = nullptr;

    TendonTailActuator* tendonActuator_ = nullptr;
    std::ofstream csv_;

    sf::Scalar elapsedTimeS_ = 0.0;
    sf::Scalar csvPeriodS_ = 0.002;
    sf::Scalar consolePeriodS_ = 0.05;
    sf::Scalar lastCsvTimeS_ = -1.0;
    sf::Scalar lastConsoleTimeS_ = -1.0;

    sf::Scalar meanStartTimeS_ = 5.0;
    sf::Scalar meanTimeS_ = 0.0;

    sf::Scalar dragImpulseFx_ = 0.0;
    sf::Scalar reactiveImpulseFx_ = 0.0;
    sf::Scalar totalImpulseFx_ = 0.0;
    sf::Scalar totalImpulseFy_ = 0.0;
    sf::Scalar totalImpulseFz_ = 0.0;
};
