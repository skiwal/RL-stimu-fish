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

class CaudalSpringActuator;

class TendonStageR2Simulator final :
    public sf::SimulationManager
{
public:
    explicit TendonStageR2Simulator(
        sf::Scalar stepsPerSecond);

    void BuildScenario() override;

    void SimulationStepCompleted(
        sf::Scalar timeStep) override;

private:
    struct Wrench {
        sf::Vector3 force{0,0,0};
        sf::Vector3 torque{0,0,0};
    };

    void BindFish();
    int FindJointIndex(const char* name) const;
    int FindLinkIndex(const char* name) const;

    void SetNeutralInitialCondition();
    void RegisterTendonActuator();
    void RegisterCaudalSpringActuator();
    void ConfigureCamera();

    sf::Vector3 GetSurfaceHydroForce(
        sf::SolidEntity* solid) const;

    sf::Vector3 GetTailSurfaceForce() const;

    Wrench GetSupportReaction() const;

    void OpenCsv();
    void RecordSample();
    void PrintTelemetry();

    sf::Robot* fishRobot_ = nullptr;
    sf::FeatherstoneRobot* fishFeatherstoneRobot_ = nullptr;
    sf::FeatherstoneEntity* fishDynamics_ = nullptr;
    sf::SolidEntity* fishBody_ = nullptr;

    int bodyLinkIndex_ = -1;
    int caudalLinkIndex_ = -1;
    int caudalJointIndex_ = -1;

    std::array<int,5> tailJointIndices_{{-1,-1,-1,-1,-1}};
    std::array<int,5> tailLinkIndices_{{-1,-1,-1,-1,-1}};

    std::array<sf::SolidEntity*,5>
        tailSolids_{{nullptr,nullptr,nullptr,nullptr,nullptr}};

    sf::SolidEntity* caudalFin_ = nullptr;

    TendonTailActuator* tendonActuator_ = nullptr;
    CaudalSpringActuator* caudalSpringActuator_ = nullptr;

    std::ofstream csv_;

    sf::Scalar elapsedTimeS_ = 0.0;

    sf::Scalar csvPeriodS_ = 0.002;
    sf::Scalar consolePeriodS_ = 0.05;

    sf::Scalar lastCsvTimeS_ = -1.0;
    sf::Scalar lastConsoleTimeS_ = -1.0;

    sf::Scalar meanStartTimeS_ = 5.0;
    sf::Scalar meanAccumTimeS_ = 0.0;

    sf::Scalar thrustImpulseNs_ = 0.0;
    sf::Scalar tailSurfaceImpulseNs_ = 0.0;

    Wrench lastSupport_;
    sf::Vector3 lastTailSurface_{0,0,0};
    sf::Scalar lastLoadcellThrustFx_ = 0.0;
};
