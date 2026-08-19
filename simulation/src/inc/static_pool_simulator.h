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
}


/*
    ================================================================
    Pool test modes

    Kept for compatibility with pool_main.cpp and later stages.

    Stage R1 itself is always a passive-tail experiment.
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

    Stage R1-A
    Five-passive-joint tail topology validation
    ================================================================

    PURPOSE

        Rebuild the tail topology before implementing tendons.

    BODY

        fixed = true

    MOTOR

        M1 is NOT part of this experiment.

        There must be no active actuator on TailJoint0.

    TENDONS

        absent

    FLEXIBLE SPINE SURROGATE

        TailJoint0
        TailJoint1
        TailJoint2
        TailJoint3
        TailJoint4

        are ALL passive revolute joints.

        For every joint i:

            tau_i =
                -k * q_i
                -c * qDot_i

    INITIAL R1-A PARAMETERS

        k = 0.65 Nm/rad

            This is the fishsim reference value.
            It is NOT yet accepted as the final Stonefish value.

        c = 0.0 Nms/rad

        emergency torque clamp = +/-0.20 Nm

    INITIAL DEFLECTION

        J0 = 2.0 deg
        J1 = 1.6 deg
        J2 = 1.2 deg
        J3 = 0.8 deg
        J4 = 0.4 deg

        qDot = 0 for all joints

    PHYSICS

        2000 Hz

    CSV

        tail_decay_stage_r1.csv
        500 Hz logging

    IMPORTANT

        Stage R1-A is NOT yet the final stiffness calibration.

        First we verify:

            1. TailJoint0 is really passive.
            2. All five joints participate.
            3. No active motor remains in the mechanical chain.
            4. The system is numerically stable.
            5. We obtain a baseline natural response for k = 0.65.

        Tail mass/inertia distribution will be checked before the
        final 3.5 Hz stiffness calibration.
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
    // Passive five-joint flexible-spine surrogate
    // ============================================================

    void RegisterPassiveTailSpringActuator();


    void ApplyInitialTailDeflection();


    void ReadTailState(
        std::array<sf::Scalar, 5>& position,
        std::array<sf::Scalar, 5>& velocity,
        std::array<sf::Scalar, 5>& rawTorque,
        std::array<sf::Scalar, 5>& appliedTorque) const;


    // ============================================================
    // Camera
    // ============================================================

    void ConfigureCamera();


    // ============================================================
    // Telemetry
    // ============================================================

    void OpenDecayCsv();


    void RecordDecaySample();


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
    // Passive tail joint indices
    //
    // [0] TailJoint0
    // [1] TailJoint1
    // [2] TailJoint2
    // [3] TailJoint3
    // [4] TailJoint4
    // ============================================================

    std::array<int, 5>
        passiveTailJointIndices_ =
        {
            -1,
            -1,
            -1,
            -1,
            -1
        };


    // ============================================================
    // Stage R1-A baseline passive parameters
    //
    // IMPORTANT:
    //
    // 0.65 is the reference fishsim hinge stiffness.
    //
    // We are deliberately starting here instead of copying the old
    // k = 22 value, because:
    //
    //     old Stage 4C:
    //         J0 locked
    //         J1~J4 passive
    //
    //     Stage R1:
    //         J0~J4 all passive
    //
    // These are different mechanical systems.
    // ============================================================

    sf::Scalar passiveTailStiffness_ =
        0.65;


    /*
        Start at the fishsim reference damping.

        Damping will be identified only AFTER stiffness/mass
        consistency is established.
    */
    sf::Scalar passiveTailDamping_ =
        0.0;


    /*
        Numerical emergency protection only.

        This must NOT be active during normal free decay.
    */
    sf::Scalar passiveTailMaxTorqueNm_ =
        0.20;


    // ============================================================
    // Initial passive-tail deflection
    //
    // Smooth cantilever-like curvature:
    //
    // root curvature larger,
    // distal curvature smaller.
    //
    // At k = 0.65 these values are far below the torque clamp.
    // ============================================================

    std::array<sf::Scalar, 5>
        initialTailDeflectionDeg_ =
        {
            2.0,
            1.6,
            1.2,
            0.8,
            0.4
        };


    // ============================================================
    // Time
    // ============================================================

    sf::Scalar elapsedTime_ =
        0.0;


    // ============================================================
    // CSV
    // ============================================================

    std::ofstream decayCsv_;


    /*
        500 Hz logging.

        Physics:
            2000 Hz

        So one CSV row every 4 physics steps.
    */
    sf::Scalar csvPeriod_ =
        0.002;


    sf::Scalar lastCsvTime_ =
        -1.0;


    std::size_t csvSampleCount_ =
        0;


    // ============================================================
    // Console
    // ============================================================

    /*
        20 Hz console telemetry.
    */
    sf::Scalar consolePeriod_ =
        0.05;


    sf::Scalar lastConsoleTime_ =
        -1.0;
};
