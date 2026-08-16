#include "inc/static_pool_simulator.h"

#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>


namespace
{

constexpr sf::Scalar kPi =
    3.14159265358979323846;


sf::Scalar DegToRad(
    sf::Scalar degrees)
{
    return
        degrees
        * kPi
        / 180.0;
}

} // namespace


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


StaticPoolSimulator::StaticPoolSimulator(
    sf::Scalar stepsPerSecond,
    PoolTestMode mode)
    : sf::SimulationManager(
          stepsPerSecond),
      mode_(mode)
{
    /*
        需要每个 physics step 更新尾部运动命令。
    */
    setCallSimulationStepCompleted(
        true);
}


void StaticPoolSimulator::BuildScenario()
{
    // ============================================================
    // App
    // ============================================================

    sf::SimulationApp* app =
        sf::SimulationApp::getApp();


    if (app == nullptr)
    {
        throw std::runtime_error(
            "Stonefish SimulationApp is not available.");
    }


    // ============================================================
    // Static pool
    // ============================================================

    const std::string scenarioPath =
        app->getDataPath()
        + "env/static_pool.scn";


    sf::ScenarioParser parser(
        this);


    if (!parser.Parse(
            scenarioPath))
    {
        throw std::runtime_error(
            "Failed to parse scenario: "
            + scenarioPath);
    }


    // ============================================================
    // Bind virtual robot hardware
    // ============================================================

    fish_.Bind(
        *this,
        "BionicFish");


    // ============================================================
    // Camera follows Body
    // ============================================================

    sf::OpenGLTrackball* trackball =
        getTrackball();


    if (
        trackball != nullptr
        && fish_.GetBody() != nullptr)
    {
        trackball->GlueToMoving(
            fish_.GetBody());
    }


    // ============================================================
    // Start with all motors centred
    // ============================================================

    fish_.SetMotorCommand(
        FishMotorCommand{});


    std::cout
        << "\n"
        << "=============================================\n"
        << " Bionic Fish V1\n"
        << " Static Pool Test Bench\n"
        << "=============================================\n"
        << "Test mode : "
        << PoolTestModeName(mode_)
        << "\n"
        << "Physics   : "
        << getStepsPerSecond()
        << " Hz\n"
        << "Motors    : 3\n"
        << "  - TailServo\n"
        << "  - LeftPectoralServo\n"
        << "  - RightPectoralServo\n"
        << "Sensors   :\n"
        << "  - IMU\n"
        << "  - Pressure\n"
        << "  - ForwardSonar\n"
        << "  - 3 x Encoder\n"
        << "=============================================\n\n";
}


void StaticPoolSimulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    if (!fish_.IsBound())
    {
        return;
    }


    elapsedTime_ +=
        timeStep;


    /*
        开始后先保持 1 秒钟中立状态。

        让：
            multibody
            buoyancy
            hydrodynamics
            sensors

        稳定下来。
    */
    constexpr sf::Scalar
        warmupTime =
            1.0;


    FishMotorCommand command;


    if (
        elapsedTime_
        < warmupTime)
    {
        command =
            FishMotorCommand{};
    }
    else if (
        mode_
        == PoolTestMode::External)
    {
        command =
            externalCommand_;
    }
    else
    {
        command =
            MakeScriptedCommand(
                elapsedTime_
                - warmupTime);
    }


    /*
        唯一的主动运动输入。

        没有：

            ApplyForce
            ApplyCentralForce
            position += ...
            fake propulsion
    */
    fish_.SetMotorCommand(
        command);


    // ============================================================
    // Telemetry
    // ============================================================

    constexpr sf::Scalar
        telemetryPeriod =
            0.50;


    if (
        lastTelemetryTime_ < 0.0
        ||
        elapsedTime_
        - lastTelemetryTime_
        >= telemetryPeriod)
    {
        PrintTelemetry();

        lastTelemetryTime_ =
            elapsedTime_;
    }
}


FishMotorCommand
StaticPoolSimulator::MakeScriptedCommand(
    sf::Scalar testTime) const
{
    FishMotorCommand command;


    /*
        ------------------------------------------------------------
        Tail propulsion parameters
        ------------------------------------------------------------

        第一轮先使用：

            amplitude = 25°
            frequency = 1.5 Hz

        θ(t) = A sin(2πft)
    */

    const sf::Scalar tailAmplitude =
        DegToRad(
            25.0);


    const sf::Scalar tailFrequency =
        1.5;


    const sf::Scalar tailWave =
        tailAmplitude
        * std::sin(

            2.0
            * kPi
            * tailFrequency
            * testTime);


    /*
        转弯时增加尾部平均偏置。

        25° wave + 10° bias
        最大约 35°，
        仍在 ±40° joint limit 内。
    */

    const sf::Scalar steeringBias =
        DegToRad(
            10.0);


    /*
        胸鳍初始测试角度。
    */
    const sf::Scalar pectoralAngle =
        DegToRad(
            12.0);


    switch (mode_)
    {
    // ------------------------------------------------------------
    // Neutral
    // ------------------------------------------------------------

    case PoolTestMode::Neutral:
    {
        break;
    }


    // ------------------------------------------------------------
    // Straight
    // ------------------------------------------------------------

    case PoolTestMode::Straight:
    {
        command.tailTargetRad =
            tailWave;

        break;
    }


    // ------------------------------------------------------------
    // Turn
    // ------------------------------------------------------------

    case PoolTestMode::TurnLeft:
    {
        command.tailTargetRad =
            tailWave
            + steeringBias;

        break;
    }


    case PoolTestMode::TurnRight:
    {
        command.tailTargetRad =
            tailWave
            - steeringBias;

        break;
    }


    // ------------------------------------------------------------
    // Dive / rise
    // ------------------------------------------------------------

    case PoolTestMode::Dive:
    {
        command.tailTargetRad =
            tailWave;


        command.leftPectoralTargetRad =
            pectoralAngle;


        command.rightPectoralTargetRad =
            pectoralAngle;

        break;
    }


    case PoolTestMode::Rise:
    {
        command.tailTargetRad =
            tailWave;


        command.leftPectoralTargetRad =
            -pectoralAngle;


        command.rightPectoralTargetRad =
            -pectoralAngle;

        break;
    }


    // ------------------------------------------------------------
    // Roll
    // ------------------------------------------------------------

    case PoolTestMode::RollLeft:
    {
        command.tailTargetRad =
            tailWave;


        command.leftPectoralTargetRad =
            pectoralAngle;


        command.rightPectoralTargetRad =
            -pectoralAngle;

        break;
    }


    case PoolTestMode::RollRight:
    {
        command.tailTargetRad =
            tailWave;


        command.leftPectoralTargetRad =
            -pectoralAngle;


        command.rightPectoralTargetRad =
            pectoralAngle;

        break;
    }


    // ------------------------------------------------------------
    // External
    // ------------------------------------------------------------

    case PoolTestMode::External:
    {
        /*
            这个 case 理论上不会到这里。

            External command 在
            SimulationStepCompleted()
            单独处理。
        */
        break;
    }
    }


    return command;
}


// ================================================================
// External interface
// ================================================================

void StaticPoolSimulator::SetExternalMotorCommand(
    const FishMotorCommand& command)
{
    externalCommand_ =
        command;
}


FishSensorData
StaticPoolSimulator::ReadFishSensors() const
{
    return fish_.ReadSensors();
}


FishMotorFeedback
StaticPoolSimulator::ReadFishMotorFeedback() const
{
    return fish_.ReadMotorFeedback();
}


BionicFish&
StaticPoolSimulator::GetFish()
{
    return fish_;
}


const BionicFish&
StaticPoolSimulator::GetFish() const
{
    return fish_;
}


// ================================================================
// Telemetry
// ================================================================

void StaticPoolSimulator::PrintTelemetry()
{
    const FishSensorData sensors =
        fish_.ReadSensors();


    const FishMotorFeedback motors =
        fish_.ReadMotorFeedback();


    sf::SolidEntity* body =
        fish_.GetBody();


    /*
        下面的 position / velocity 是 DEBUG GROUND TRUTH。

        只用于判断：
            鱼到底有没有真正移动。

        不属于未来 RL observation。
    */

    sf::Vector3 position(
        0.0,
        0.0,
        0.0);


    sf::Vector3 velocity(
        0.0,
        0.0,
        0.0);


    if (body != nullptr)
    {
        position =
            body
                ->getCGTransform()
                .getOrigin();


        velocity =
            body
                ->getLinearVelocity();
    }


    sf::Scalar centerSonar =
        -1.0;


    if (
        !sensors
             .sonarRangesMeters
             .empty())
    {
        centerSonar =
            sensors
                .sonarRangesMeters[
                    sensors
                        .sonarRangesMeters
                        .size()
                    / 2];
    }


    std::cout
        << std::fixed
        << std::setprecision(3)


        << "[FishV1] "


        << "t="
        << elapsedTime_


        // --------------------------------------------------------
        // Debug ground truth
        // --------------------------------------------------------

        << " | pos=("
        << position.getX()
        << ", "
        << position.getY()
        << ", "
        << position.getZ()
        << ")"


        << " vel=("
        << velocity.getX()
        << ", "
        << velocity.getY()
        << ", "
        << velocity.getZ()
        << ")"


        // --------------------------------------------------------
        // Virtual IMU
        // --------------------------------------------------------

        << " | IMU rpy=("
        << sensors.rollRad
        << ", "
        << sensors.pitchRad
        << ", "
        << sensors.yawRad
        << ")"


        // --------------------------------------------------------
        // Depth
        // --------------------------------------------------------

        << " | depth="
        << sensors.depthMeters
        << "m"


        // --------------------------------------------------------
        // Tail
        // --------------------------------------------------------

        << " | tail="
        << motors.tail.targetPositionRad
        << "/"
        << motors.tail.positionRad
        << "rad"


        // --------------------------------------------------------
        // Pectoral
        // --------------------------------------------------------

        << " | fins=("
        << motors
               .leftPectoral
               .positionRad
        << ", "
        << motors
               .rightPectoral
               .positionRad
        << ")"


        // --------------------------------------------------------
        // Sonar
        // --------------------------------------------------------

        << " | sonar_center="
        << centerSonar
        << "m"


        << '\n';
}
