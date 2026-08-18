#pragma once

#include <Stonefish/core/SimulationManager.h>


namespace sf
{
class Robot;
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

    Stage 4 — M1 tail propulsion smoke test
    ================================================================

    Straight mode:

        M1 / TailMotor:
            ON

        command:
            q_des(t) = A * sin(2*pi*f*t)

        A:
            20 deg

        f:
            1.2 Hz

        M2:
            OFF

        M3:
            OFF


    Neutral mode:

        M1 = OFF
        M2 = OFF
        M3 = OFF


    IMPORTANT:

        No body-level propulsion force.
        No ApplyForce().
        No fake thrust.

        Any translation of the fish must arise from:
            articulated tail motion
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

    /*
        Find BionicFish and M1.
    */
    void BindBionicFish();


    /*
        Automatically frame the fish in the GUI.
    */
    void ConfigureCamera();


    /*
        Configure motor state according to PoolTestMode.
    */
    void ConfigureMotorTest();


    /*
        Generate continuous M1 sinusoidal command.
    */
    void UpdateTailMotorCommand();


    /*
        Print M1 command and feedback.
    */
    void PrintMotorTelemetry();


    // ============================================================
    // Test mode
    // ============================================================

    PoolTestMode mode_ =
        PoolTestMode::Neutral;


    // ============================================================
    // Robot
    // ============================================================

    sf::Robot* fishRobot_ =
        nullptr;


    sf::SolidEntity* fishBody_ =
        nullptr;


    // ============================================================
    // M1
    // ============================================================

    sf::Servo* tailMotor_ =
        nullptr;


    // ============================================================
    // M1 test parameters
    //
    // These are intentionally conservative compared with the
    // available actuator capability.
    // ============================================================

    /*
        ±20 degrees

        20 deg =
        0.3490658503988659 rad
    */
    sf::Scalar tailAmplitudeRad_ =
        0.3490658503988659;


    /*
        Tail oscillation frequency.
    */
    sf::Scalar tailFrequencyHz_ =
        1.2;


    /*
        Temporary Stage-4 torque limit.

        This is NOT the final hardware limit.

        We deliberately start much lower than the full motor
        capability.
    */
    sf::Scalar tailMaxTorqueNm_ =
        1.0;


    /*
        Maximum servo angular velocity.
    */
    sf::Scalar tailMaxVelocityRadS_ =
        3.5;


    /*
        Let physics settle for one second before starting motion.
    */
    sf::Scalar driveStartTime_ =
        1.0;


    /*
        Increase amplitude gradually over one second.
    */
    sf::Scalar driveRampTime_ =
        1.0;


    // ============================================================
    // Simulation time
    // ============================================================

    sf::Scalar elapsedTime_ =
        0.0;


    // ============================================================
    // Telemetry
    // ============================================================

    sf::Scalar lastTelemetryTime_ =
        -1.0;


    /*
        20 Hz terminal output.
    */
    sf::Scalar telemetryPeriod_ =
        0.05;


    /*
        Last commanded M1 angle.
    */
    sf::Scalar lastTailCommandRad_ =
        0.0;
};
