#pragma once

#include <Stonefish/core/SimulationManager.h>

#include <array>
#include <cstddef>
#include <fstream>


namespace sf
{
class Robot;
class FeatherstoneRobot;
class FeatherstoneEntity;
class SolidEntity;
class Servo;
}


/*
    ================================================================
    Pool test modes
    ================================================================
*/
enum class PoolTestMode
{
    Neutral,

    Straight,

    TurnLeft,

    TurnRight,

    Dive,

    Rise,

    RollLeft,

    RollRight,

    External
};


const char*
PoolTestModeName(
    PoolTestMode mode);


/*
    ================================================================
    StaticPoolSimulator

    Stage 5A
    First free-swimming test
    ================================================================

    PHYSICS

        2000 Hz
        configured in pool_main.cpp

    BODY

        robot_fixed = false

    M1 / TailJoint0

        neutral:
            disabled

        straight:
            sinusoidal position command

            amplitude:
                +/-5 deg

            frequency:
                0.4 Hz

            max torque:
                0.05 Nm

            max velocity:
                0.35 rad/s

            first 1 s:
                command = 0

            second 1 s:
                amplitude ramp 0 -> 100%

    PASSIVE TAIL

        TailJoint1~4

            tau = -k*q - c*qDot

        k:
            22 Nm/rad

        c:
            0.001 Nms/rad

        torque safety clamp:
            +/-0.05 Nm

    M2/M3

        disabled

    IMPORTANT

        No body-level ApplyForce().
        No fake propulsion.

        Any swimming motion must arise from:

            M1
            +
            articulated tail
            +
            Stonefish hydrodynamics
*/
class StaticPoolSimulator final
    : public sf::SimulationManager
{
public:

    StaticPoolSimulator(
        sf::Scalar stepsPerSecond,
        PoolTestMode mode);


    void BuildScenario() override;


    void SimulationStepCompleted(
        sf::Scalar timeStep) override;


    sf::Robot*
    GetFishRobot() const;


    sf::SolidEntity*
    GetFishBody() const;


    PoolTestMode
    GetTestMode() const;


private:

    // ============================================================
    // Robot binding
    // ============================================================

    void BindBionicFish();


    int FindDynamicsJointIndex(
        const char* shortJointName) const;


    // ============================================================
    // Camera
    // ============================================================

    void ConfigureCamera();


    // ============================================================
    // Passive elastic tail
    // ============================================================

    void RegisterPassiveTailSpringActuator();


    void ReadPassiveTailState(
        std::array<sf::Scalar, 4>& position,
        std::array<sf::Scalar, 4>& velocity,
        std::array<sf::Scalar, 4>& rawTorque,
        std::array<sf::Scalar, 4>& appliedTorque) const;


    // ============================================================
    // M1 active tail drive
    // ============================================================

    void ConfigureTailDrive();


    void UpdateTailDriveCommand();


    // ============================================================
    // Safety
    // ============================================================

    void CheckSafety();


    void EmergencyDisableTailMotor(
        const char* reason);


    // ============================================================
    // Telemetry / CSV
    // ============================================================

    void OpenSwimCsv();


    void RecordSwimSample();


    void PrintTelemetry();


    // ============================================================
    // Mode
    // ============================================================

    PoolTestMode mode_ =
        PoolTestMode::Neutral;


    // ============================================================
    // Robot
    // ============================================================

    sf::Robot* fishRobot_ =
        nullptr;


    sf::FeatherstoneRobot* fishFeatherstoneRobot_ =
        nullptr;


    sf::FeatherstoneEntity* fishDynamics_ =
        nullptr;


    sf::SolidEntity* fishBody_ =
        nullptr;


    // ============================================================
    // M1
    // ============================================================

    sf::Servo* tailMotor_ =
        nullptr;


    /*
        +/-5 degrees
    */
    sf::Scalar tailAmplitudeRad_ =
        0.08726646259971647;


    /*
        First free-swimming test frequency.
    */
    sf::Scalar tailFrequencyHz_ =
        0.4;


    /*
        Conservative first-test torque limit.
    */
    sf::Scalar tailMaxTorqueNm_ =
        0.05;


    /*
        M1 maximum angular velocity.
    */
    sf::Scalar tailMaxVelocityRadS_ =
        0.35;


    /*
        First second:
            M1 stays at zero.
    */
    sf::Scalar driveStartTime_ =
        1.0;


    /*
        Next second:
            amplitude ramps from 0 to full.
    */
    sf::Scalar driveRampTime_ =
        1.0;


    /*
        Latest M1 desired position.
    */
    sf::Scalar lastTailCommandRad_ =
        0.0;


    // ============================================================
    // Passive tail indices
    //
    // [0] TailJoint1
    // [1] TailJoint2
    // [2] TailJoint3
    // [3] TailJoint4
    // ============================================================

    std::array<int, 4>
        passiveTailJointIndices_ =
        {
            -1,
            -1,
            -1,
            -1
        };


    // ============================================================
    // Calibrated passive-tail parameters
    // ============================================================

    sf::Scalar passiveTailStiffness_ =
        22.0;


    sf::Scalar passiveTailDamping_ =
        0.001;


    sf::Scalar passiveTailMaxTorqueNm_ =
        0.05;


    // ============================================================
    // Safety
    // ============================================================

    bool motorSafetyTripped_ =
        false;


    /*
        M1 command is only +/-5 deg.

        25 deg therefore indicates something is seriously wrong.
    */
    sf::Scalar safetyMaxM1PositionRad_ =
        0.4363323129985824;   // 25 deg


    sf::Scalar safetyMaxM1VelocityRadS_ =
        2.0;


    sf::Scalar safetyMaxM1TrackingErrorRad_ =
        0.3490658503988659;   // 20 deg


    /*
        Passive joint emergency limits.
    */
    sf::Scalar safetyMaxPassivePositionRad_ =
        0.3490658503988659;   // 20 deg


    sf::Scalar safetyMaxPassiveVelocityRadS_ =
        30.0;


    /*
        Whole-fish emergency velocity.
    */
    sf::Scalar safetyMaxBodySpeedMS_ =
        2.0;


    /*
        Maximum acceptable roll/pitch during Stage 5A.
    */
    sf::Scalar safetyMaxRollPitchRad_ =
        1.0471975511965976;   // 60 deg


    /*
        Pool uses NED:

            surface approximately z = 0
            bottom approximately z = 4

        These are only emergency limits.
    */
    sf::Scalar safetyMinBodyZ_ =
        0.10;


    sf::Scalar safetyMaxBodyZ_ =
        3.90;


    // ============================================================
    // Initial free-swim reference
    //
    // IMPORTANT:
    //
    // DO NOT capture this from BuildScenario().
    //
    // It is captured after the first completed physics step.
    // ============================================================

    sf::Scalar initialBodyX_ =
        0.0;


    sf::Scalar initialBodyY_ =
        0.0;


    sf::Scalar initialBodyZ_ =
        0.0;


    bool initialBodyStateCaptured_ =
        false;


    // ============================================================
    // Simulation time
    // ============================================================

    sf::Scalar elapsedTime_ =
        0.0;


    // ============================================================
    // CSV
    // ============================================================

    std::ofstream swimCsv_;


    /*
        200 Hz telemetry.

        Physics is 2000 Hz.
    */
    sf::Scalar csvPeriod_ =
        0.005;


    sf::Scalar lastCsvTime_ =
        -1.0;


    std::size_t csvSampleCount_ =
        0;


    // ============================================================
    // Console telemetry
    // ============================================================

    /*
        10 Hz console output.
    */
    sf::Scalar consolePeriod_ =
        0.10;


    sf::Scalar lastConsoleTime_ =
        -1.0;
};
