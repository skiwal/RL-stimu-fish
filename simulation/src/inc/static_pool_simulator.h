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
    Static-pool test modes
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

    Stage 4C — Passive Tail Free-Decay Calibration
    ================================================================

    PURPOSE

        Measure the actual dynamic response of:

            TailJoint1
            TailJoint2
            TailJoint3
            TailJoint4

        under the real Stonefish multibody + underwater physics.

    BOUNDARY CONDITION

        Body:
            fixed=true

        TailJoint0 / M1:
            position hold at 0 deg

        M1 does NOT oscillate.

        This makes TailJoint0 approximately a fixed root boundary
        for the passive tail.

    PASSIVE JOINT MODEL

        For TailJoint1~4:

            tau = -k*q - c*qDot

        Initial bootstrap values:

            k = 0.02 Nm/rad
            c = 0.001 Nms/rad

    INITIAL CONDITION

        TailJoint1 = +8 deg
        TailJoint2 = +6 deg
        TailJoint3 = +4 deg
        TailJoint4 = +2 deg

        all initial angular velocities = 0

    DATA OUTPUT

        tail_decay_stage4c.csv

        CSV rate:
            100 Hz

        Console rate:
            20 Hz

    IMPORTANT

        No Body ApplyForce().
        No fake propulsion.
        No periodic motor command.
        M2/M3 remain disabled.
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
    // Binding
    // ============================================================

    void BindBionicFish();


    int FindDynamicsJointIndex(
        const char* shortJointName) const;


    // ============================================================
    // Camera
    // ============================================================

    void ConfigureCamera();


    // ============================================================
    // Passive tail
    // ============================================================

    void RegisterPassiveTailSpringActuator();


    void ApplyInitialTailDeflection();


    // ============================================================
    // M1 root clamp
    // ============================================================

    void ConfigureTailRootHold();


    // ============================================================
    // Data
    // ============================================================

    void OpenDecayCsv();


    void RecordDecaySample();


    void PrintDecayTelemetry();


    void ReadPassiveTailState(
        std::array<sf::Scalar, 4>& position,
        std::array<sf::Scalar, 4>& velocity,
        std::array<sf::Scalar, 4>& torque) const;


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
    //
    // Stage 4C:
    //
    // M1 is NOT generating a waveform.
    //
    // It only holds TailJoint0 at zero.
    // ============================================================

    sf::Servo* tailMotor_ =
        nullptr;


    sf::Scalar rootHoldTorqueNm_ =
        0.05;


    sf::Scalar rootHoldMaxVelocityRadS_ =
        0.35;


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
    // Passive tail parameters
    //
    // NOT final calibrated values.
    // ============================================================

    sf::Scalar passiveTailStiffness_ =
        22.0;


    sf::Scalar passiveTailDamping_ =
        0.001;


    /*
        Numerical safety clamp.

        This is not intended to define the final mechanical system.
    */
    sf::Scalar passiveTailMaxTorqueNm_ =
        0.05;


    // ============================================================
    // Initial deflection
    //
    // degrees
    // ============================================================

    std::array<sf::Scalar, 4>
        initialTailDeflectionDeg_ =
        {
            0.0727,
            0.0545,
            0.0364,
            0.0182
        };


    // ============================================================
    // Time
    // ============================================================

    sf::Scalar elapsedTime_ =
        0.0;


    /*
        Around 6 seconds is already plenty for the first
        calibration experiment.

        The simulator itself will continue running after that.
    */
    sf::Scalar decayMeasurementDuration_ =
        6.0;


    bool decayCompletionAnnounced_ =
        false;


    // ============================================================
    // CSV
    // ============================================================

    std::ofstream decayCsv_;


    sf::Scalar csvPeriod_ =
        0.01;       // 100 Hz


    sf::Scalar lastCsvTime_ =
        -1.0;


    std::size_t csvSampleCount_ =
        0;


    // ============================================================
    // Console telemetry
    // ============================================================

    sf::Scalar consolePeriod_ =
        0.05;       // 20 Hz


    sf::Scalar lastConsoleTime_ =
        -1.0;
};
