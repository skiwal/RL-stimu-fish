#pragma once

#include <Stonefish/core/SimulationManager.h>

#include "tendon_tail_actuator.h"

#include <array>
#include <fstream>


namespace sf
{
class Robot;
class FeatherstoneRobot;
class FeatherstoneEntity;
class SolidEntity;
class Servo;
}


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

    void BindFish();

    int FindJointIndex(
        const char* name) const;

    int FindLinkIndex(
        const char* name) const;

    void SetNeutralInitialCondition();

    void ConfigureMotor();

    void RegisterTendonActuator();

    void ConfigureCamera();

    void OpenCsv();

    void RecordSample();

    void PrintTelemetry();


    sf::Robot* fishRobot_ = nullptr;

    sf::FeatherstoneRobot*
        fishFeatherstoneRobot_ = nullptr;

    sf::FeatherstoneEntity*
        fishDynamics_ = nullptr;

    sf::SolidEntity*
        fishBody_ = nullptr;

    sf::Servo*
        motorServo_ = nullptr;


    int bodyLinkIndex_ = -1;

    int motorLinkIndex_ = -1;
    int motorJointIndex_ = -1;

    std::array<int,5>
        tailJointIndices_{
            {-1,-1,-1,-1,-1}
        };

    std::array<int,5>
        tailLinkIndices_{
            {-1,-1,-1,-1,-1}
        };


    TendonTailActuator*
        tendonActuator_ = nullptr;


    std::ofstream csv_;

    sf::Scalar elapsedTimeS_ = 0.0;

    sf::Scalar csvPeriodS_ = 0.001;
    sf::Scalar consolePeriodS_ = 0.05;

    sf::Scalar lastCsvTimeS_ = -1.0;
    sf::Scalar lastConsoleTimeS_ = -1.0;


    // --------------------------------------------------------
    // Source fishsim actuation experiment.
    //
    // f = 1.25 Hz
    // omega = 2*pi*f
    // --------------------------------------------------------

    sf::Scalar motorFrequencyHz_ = 1.25;

    sf::Scalar motorTargetVelocityRadS_ =
        7.853981633974483;

    sf::Scalar motorMaxVelocityRadS_ =
        31.41592653589793;

    sf::Scalar motorMaxTorqueNm_ =
        100.0;
};
