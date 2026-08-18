#include "inc/static_pool_simulator.h"

#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/Servo.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>


// ================================================================
// Test mode name
// ================================================================

const char*
PoolTestModeName(
    PoolTestMode mode)
{
    switch (mode)
    {
    case PoolTestMode::Neutral:
        return "neutral";


    case PoolTestMode::Straight:
        return "straight";


    case PoolTestMode::TurnLeft:
        return "turn_left";


    case PoolTestMode::TurnRight:
        return "turn_right";


    case PoolTestMode::Dive:
        return "dive";


    case PoolTestMode::Rise:
        return "rise";


    case PoolTestMode::RollLeft:
        return "roll_left";


    case PoolTestMode::RollRight:
        return "roll_right";


    case PoolTestMode::External:
        return "external";
    }


    return "unknown";
}


// ================================================================
// Constructor
// ================================================================

StaticPoolSimulator::StaticPoolSimulator(
    sf::Scalar stepsPerSecond,
    PoolTestMode mode)
    : sf::SimulationManager(
          stepsPerSecond),
      mode_(mode)
{
    /*
        We need SimulationStepCompleted() because we continuously
        update the Servo desired-position setpoint.

        This is different from applying DriveJoint() torque here.

        setDesiredPosition() changes persistent actuator state.
        Stonefish's Servo will consume the latest setpoint during
        its actuator Update() stage.
    */

    setCallSimulationStepCompleted(
        true);
}


// ================================================================
// Build scenario
// ================================================================

void StaticPoolSimulator::BuildScenario()
{
    // ============================================================
    // Stonefish application
    // ============================================================

    sf::SimulationApp* app =
        sf::SimulationApp::getApp();


    if (
        app == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "SimulationApp is not available.");
    }


    // ============================================================
    // Validated pool scenario
    // ============================================================

    const std::string scenarioPath =
        app->getDataPath()
        + "env/static_pool.scn";


    std::cout
        << "\n"
        << "Loading scenario:\n"
        << "  "
        << scenarioPath
        << "\n"
        << std::endl;


    // ============================================================
    // Parse XML
    // ============================================================

    sf::ScenarioParser parser(
        this);


    if (
        !parser.Parse(
            scenarioPath))
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "failed to parse scenario: "
            + scenarioPath);
    }


    // ============================================================
    // Bind robot and M1
    // ============================================================

    BindBionicFish();


    // ============================================================
    // Configure GUI camera
    // ============================================================

    ConfigureCamera();


    // ============================================================
    // Configure active motor test
    // ============================================================

    ConfigureMotorTest();


    // ============================================================
    // Startup information
    // ============================================================

    std::cout
        << "\n"
        << "============================================================\n"
        << " BionicFish V1 - Stage 4 M1 Propulsion Smoke Test\n"
        << "============================================================\n"
        << "\n"
        << "Scenario:\n"
        << "  env/static_pool.scn\n"
        << "\n"
        << "Robot:\n"
        << "  BionicFish\n"
        << "\n"
        << "Mode:\n"
        << "  "
        << PoolTestModeName(
               mode_)
        << "\n"
        << "\n";


    if (
        mode_
        == PoolTestMode::Straight)
    {
        std::cout
            << "M1 TailMotor:\n"
            << "  enabled                 YES\n"
            << "  control mode            POSITION\n"
            << "  waveform                sine\n"
            << "  amplitude               +/-20 deg\n"
            << "  frequency               "
            << tailFrequencyHz_
            << " Hz\n"
            << "  max torque              "
            << tailMaxTorqueNm_
            << " Nm\n"
            << "  max velocity            "
            << tailMaxVelocityRadS_
            << " rad/s\n"
            << "  start delay             "
            << driveStartTime_
            << " s\n"
            << "  amplitude ramp          "
            << driveRampTime_
            << " s\n";
    }
    else
    {
        std::cout
            << "M1 TailMotor:\n"
            << "  enabled                 NO\n";
    }


    std::cout
        << "\n"
        << "M2 LeftPectoralMotor:\n"
        << "  enabled                 NO\n"
        << "\n"
        << "M3 RightPectoralMotor:\n"
        << "  enabled                 NO\n"
        << "\n"
        << "Passive C++ tail spring:\n"
        << "  enabled                 NO\n"
        << "\n"
        << "Body-level ApplyForce:\n"
        << "  used                    NO\n"
        << "\n"
        << "Camera:\n"
        << "  tracking BionicFish/Body\n"
        << "\n"
        << "Expected smoke-test result:\n"
        << "  TailJoint0 oscillates continuously.\n"
        << "  Hydrodynamic reaction may move the fish.\n"
        << "============================================================\n"
        << std::endl;
}


// ================================================================
// Bind robot and actuator
// ================================================================

void StaticPoolSimulator::BindBionicFish()
{
    // ============================================================
    // Robot
    // ============================================================

    fishRobot_ =
        getRobot(
            "BionicFish");


    if (
        fishRobot_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "robot 'BionicFish' was not found.");
    }


    // ============================================================
    // Base body
    // ============================================================

    fishBody_ =
        fishRobot_
            ->getBaseLink();


    if (
        fishBody_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "BionicFish Body was not found.");
    }


    // ============================================================
    // Find TailMotor
    //
    // Stonefish's scenario parser normally namespaces robot
    // actuators:
    //
    //     BionicFish/TailMotor
    //
    // Keep a short-name fallback for compatibility.
    // ============================================================

    sf::Actuator* actuator =
        fishRobot_
            ->getActuator(
                "BionicFish/TailMotor");


    if (
        actuator == nullptr)
    {
        actuator =
            fishRobot_
                ->getActuator(
                    "TailMotor");
    }


    if (
        actuator == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "TailMotor actuator was not found.");
    }


    tailMotor_ =
        dynamic_cast<sf::Servo*>(
            actuator);


    if (
        tailMotor_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: "
            "TailMotor exists but is not a Stonefish Servo.");
    }


    std::cout
        << "\n"
        << "BionicFish binding complete:\n"
        << "  Robot       = OK\n"
        << "  Body        = OK\n"
        << "  TailMotor   = OK\n"
        << std::endl;
}


// ================================================================
// Camera
// ================================================================

void StaticPoolSimulator::ConfigureCamera()
{
    if (
        fishBody_ == nullptr)
    {
        return;
    }


    sf::OpenGLTrackball* trackball =
        getTrackball();


    if (
        trackball == nullptr)
    {
        return;
    }


    // Follow fish body continuously.
    trackball->GlueToMoving(
        fishBody_);


    // Center immediately on fish.
    trackball->UpdateCenterPos();


    // Zoom from Stonefish's wide default view toward the fish.
    trackball->MouseScroll(
        -10.5f);


    // Apply immediately.
    trackball->UpdateTransform();


    std::cout
        << "Camera configured:\n"
        << "  target   = BionicFish/Body\n"
        << "  tracking = ON\n"
        << std::endl;
}


// ================================================================
// Motor setup
// ================================================================

void StaticPoolSimulator::ConfigureMotorTest()
{
    if (
        tailMotor_ == nullptr)
    {
        return;
    }


    // ============================================================
    // Always start at neutral.
    // ============================================================

    tailMotor_
        ->setControlMode(
            sf::ServoControlMode::POSITION);


    tailMotor_
        ->setDesiredPosition(
            0.0);


    // ============================================================
    // Straight mode:
    //
    // enable M1 with deliberately conservative limits.
    //
    // Other modes currently keep M1 disabled.
    // ============================================================

    if (
        mode_
        == PoolTestMode::Straight)
    {
        tailMotor_
            ->setMaxTorque(
                tailMaxTorqueNm_);


        tailMotor_
            ->setMaxVelocity(
                tailMaxVelocityRadS_);


        std::cout
            << "M1 propulsion test enabled."
            << std::endl;
    }
    else
    {
        // Safe state.
        tailMotor_
            ->setMaxTorque(
                0.0);


        std::cout
            << "M1 remains disabled."
            << std::endl;
    }
}


// ================================================================
// Per-step callback
// ================================================================

void StaticPoolSimulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTime_ +=
        timeStep;


    // ============================================================
    // Update persistent M1 servo setpoint.
    // ============================================================

    UpdateTailMotorCommand();


    // ============================================================
    // Telemetry
    // ============================================================

    if (
        lastTelemetryTime_ < 0.0
        ||
        (
            elapsedTime_
            - lastTelemetryTime_
        )
        >= telemetryPeriod_)
    {
        PrintMotorTelemetry();


        lastTelemetryTime_ =
            elapsedTime_;
    }
}


// ================================================================
// M1 sinusoidal command
// ================================================================

void StaticPoolSimulator::UpdateTailMotorCommand()
{
    if (
        tailMotor_ == nullptr)
    {
        return;
    }


    // ============================================================
    // Only Straight mode drives M1.
    // ============================================================

    if (
        mode_
        != PoolTestMode::Straight)
    {
        lastTailCommandRad_ =
            0.0;


        tailMotor_
            ->setDesiredPosition(
                0.0);


        return;
    }


    // ============================================================
    // First second:
    //
    // remain exactly neutral.
    // ============================================================

    if (
        elapsedTime_
        < driveStartTime_)
    {
        lastTailCommandRad_ =
            0.0;


        tailMotor_
            ->setDesiredPosition(
                0.0);


        return;
    }


    // ============================================================
    // Time since oscillation started.
    // ============================================================

    const sf::Scalar driveTime =
        elapsedTime_
        - driveStartTime_;


    // ============================================================
    // Smooth linear amplitude ramp:
    //
    // 0 -> 1 over driveRampTime_.
    // ============================================================

    sf::Scalar ramp =
        1.0;


    if (
        driveRampTime_
        > 0.0)
    {
        ramp =
            driveTime
            / driveRampTime_;
    }


    ramp =
        std::max(
            sf::Scalar(0.0),
            std::min(
                sf::Scalar(1.0),
                ramp));


    // ============================================================
    // Sinusoidal M1 trajectory
    //
    //     q_des =
    //
    //       ramp
    //       *
    //       amplitude
    //       *
    //       sin(2*pi*f*t)
    //
    // ============================================================

    constexpr sf::Scalar pi =
        3.14159265358979323846;


    const sf::Scalar omega =
        2.0
        * pi
        * tailFrequencyHz_;


    const sf::Scalar phase =
        omega
        * driveTime;


    const sf::Scalar command =
        ramp
        * tailAmplitudeRad_
        * std::sin(
            phase);


    lastTailCommandRad_ =
        command;


    // ============================================================
    // IMPORTANT:
    //
    // This is a REAL joint actuator setpoint.
    //
    // It does not teleport the joint.
    // It does not directly set joint state.
    // It does not apply a force to the Body.
    //
    // Stonefish Servo has to physically track this setpoint subject
    // to torque and velocity limits.
    // ============================================================

    tailMotor_
        ->setDesiredPosition(
            command);
}


// ================================================================
// Motor telemetry
// ================================================================

void StaticPoolSimulator::PrintMotorTelemetry()
{
    if (
        tailMotor_ == nullptr)
    {
        return;
    }


    constexpr sf::Scalar radToDeg =
        180.0
        / 3.14159265358979323846;


    const sf::Scalar desired =
        lastTailCommandRad_;


    const sf::Scalar actual =
        tailMotor_
            ->getPosition();


    const sf::Scalar velocity =
        tailMotor_
            ->getVelocity();


    const sf::Scalar effort =
        tailMotor_
            ->getEffort();


    std::cout
        << std::fixed
        << std::setprecision(
            4)

        << "[M1] "

        << "t="
        << elapsedTime_

        << " | cmd="
        << desired
        << " rad"

        << " ("
        << desired
               * radToDeg
        << " deg)"

        << " | q="
        << actual
        << " rad"

        << " ("
        << actual
               * radToDeg
        << " deg)"

        << " | qDot="
        << velocity
        << " rad/s"

        << " | effort="
        << effort
        << " Nm"

        << std::endl;
}


// ================================================================
// Getters
// ================================================================

sf::Robot*
StaticPoolSimulator::GetFishRobot() const
{
    return fishRobot_;
}


sf::SolidEntity*
StaticPoolSimulator::GetFishBody() const
{
    return fishBody_;
}


PoolTestMode
StaticPoolSimulator::GetTestMode() const
{
    return mode_;
}
