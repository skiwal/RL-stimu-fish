#include "inc/bionic_fish.h"

#include <Stonefish/core/Robot.h>
#include <Stonefish/core/SimulationManager.h>

#include <Stonefish/actuators/Servo.h>

#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/forcefields/Ocean.h>

#include <Stonefish/sensors/ScalarSensor.h>

#include <algorithm>
#include <stdexcept>
#include <string>


namespace
{

template <typename T, typename U>
T* RequireType(
    U* object,
    const std::string& name)
{
    T* result =
        dynamic_cast<T*>(
            object);

    if (result == nullptr)
    {
        throw std::runtime_error(
            "BionicFish: required object '"
            + name
            + "' was not found or has the wrong type.");
    }

    return result;
}


sf::Scalar ReadChannel(
    const sf::ScalarSensor* sensor,
    unsigned int channel)
{
    if (sensor == nullptr)
    {
        return 0.0;
    }

    if (
        channel
        >= sensor->getNumOfChannels())
    {
        return 0.0;
    }

    /*
        Stonefish getLastValue() returns 0
        before the first valid sample exists.
    */
    return sensor->getLastValue(
        channel);
}

} // namespace


void BionicFish::Bind(
    sf::SimulationManager& simulation,
    const std::string& robotName)
{
    robotName_ =
        robotName;


    // ============================================================
    // Robot
    // ============================================================

    robot_ =
        simulation.getRobot(
            robotName_);


    if (robot_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish: robot '"
            + robotName_
            + "' was not found.");
    }


    body_ =
        robot_->getBaseLink();


    if (body_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish: base link was not found.");
    }


    const std::string prefix =
        robotName_
        + "/";


    // ============================================================
    // Motors
    // ============================================================

    tailServo_ =
        RequireType<sf::Servo>(
            simulation.getActuator(
                prefix
                + "TailServo"),

            prefix
            + "TailServo");


    leftPectoralServo_ =
        RequireType<sf::Servo>(
            simulation.getActuator(
                prefix
                + "LeftPectoralServo"),

            prefix
            + "LeftPectoralServo");


    rightPectoralServo_ =
        RequireType<sf::Servo>(
            simulation.getActuator(
                prefix
                + "RightPectoralServo"),

            prefix
            + "RightPectoralServo");


    /*
        明确使用 position control。

        ScenarioParser 已经会默认这样初始化，
        这里再次显式设置，方便以后阅读。
    */
    tailServo_->setControlMode(
        sf::ServoControlMode::POSITION);

    leftPectoralServo_->setControlMode(
        sf::ServoControlMode::POSITION);

    rightPectoralServo_->setControlMode(
        sf::ServoControlMode::POSITION);


    // ============================================================
    // Sensors
    // ============================================================

    imu_ =
        RequireType<sf::ScalarSensor>(
            simulation.getSensor(
                prefix
                + "IMU"),

            prefix
            + "IMU");


    pressure_ =
        RequireType<sf::ScalarSensor>(
            simulation.getSensor(
                prefix
                + "Pressure"),

            prefix
            + "Pressure");


    sonar_ =
        RequireType<sf::ScalarSensor>(
            simulation.getSensor(
                prefix
                + "ForwardSonar"),

            prefix
            + "ForwardSonar");


    tailEncoder_ =
        RequireType<sf::ScalarSensor>(
            simulation.getSensor(
                prefix
                + "TailEncoder"),

            prefix
            + "TailEncoder");


    leftPectoralEncoder_ =
        RequireType<sf::ScalarSensor>(
            simulation.getSensor(
                prefix
                + "LeftPectoralEncoder"),

            prefix
            + "LeftPectoralEncoder");


    rightPectoralEncoder_ =
        RequireType<sf::ScalarSensor>(
            simulation.getSensor(
                prefix
                + "RightPectoralEncoder"),

            prefix
            + "RightPectoralEncoder");


    // ============================================================
    // Pressure -> depth constants
    // ============================================================

    sf::Ocean* ocean =
        simulation.getOcean();


    if (ocean != nullptr)
    {
        const sf::Fluid liquid =
            ocean->getLiquid();

        if (liquid.density > 0.0)
        {
            waterDensity_ =
                liquid.density;
        }
    }


    const sf::Vector3 gravity =
        simulation.getGravity();


    const sf::Scalar gravityLength =
        gravity.length();


    if (gravityLength > 0.0)
    {
        gravityMagnitude_ =
            gravityLength;
    }


    // ============================================================
    // Initial motor command
    // ============================================================

    command_ =
        FishMotorCommand{};


    SetMotorCommand(
        command_);
}


bool BionicFish::IsBound() const
{
    return
        robot_ != nullptr
        && body_ != nullptr
        && tailServo_ != nullptr
        && leftPectoralServo_ != nullptr
        && rightPectoralServo_ != nullptr
        && imu_ != nullptr
        && pressure_ != nullptr
        && sonar_ != nullptr;
}


void BionicFish::EnsureBound() const
{
    if (!IsBound())
    {
        throw std::logic_error(
            "BionicFish has not been bound to a Stonefish robot.");
    }
}


// ================================================================
// Motor input
// ================================================================

void BionicFish::SetMotorCommand(
    const FishMotorCommand& command)
{
    EnsureBound();


    /*
        与 bionic_fish_v1.scn 中 joint limit 保持一致。

        Tail:
            ±40°

        Pectoral:
            ±30°
    */

    constexpr sf::Scalar
        tailLimit =
            0.6981317008;


    constexpr sf::Scalar
        pectoralLimit =
            0.5235987756;


    command_.tailTargetRad =
        std::clamp(
            command.tailTargetRad,
            -tailLimit,
            tailLimit);


    command_.leftPectoralTargetRad =
        std::clamp(
            command.leftPectoralTargetRad,
            -pectoralLimit,
            pectoralLimit);


    command_.rightPectoralTargetRad =
        std::clamp(
            command.rightPectoralTargetRad,
            -pectoralLimit,
            pectoralLimit);


    /*
        这里就是虚拟硬件的“motor write”。

        后面真实机器人版本可以对应：

            UART
            CAN
            PWM
            Dynamixel command
            etc.
    */

    tailServo_->setDesiredPosition(
        command_.tailTargetRad);


    leftPectoralServo_->setDesiredPosition(
        command_
            .leftPectoralTargetRad);


    rightPectoralServo_->setDesiredPosition(
        command_
            .rightPectoralTargetRad);
}


void BionicFish::SetTailTarget(
    sf::Scalar targetRad)
{
    FishMotorCommand command =
        command_;


    command.tailTargetRad =
        targetRad;


    SetMotorCommand(
        command);
}


void BionicFish::SetPectoralTargets(
    sf::Scalar leftRad,
    sf::Scalar rightRad)
{
    FishMotorCommand command =
        command_;


    command.leftPectoralTargetRad =
        leftRad;


    command.rightPectoralTargetRad =
        rightRad;


    SetMotorCommand(
        command);
}


FishMotorCommand
BionicFish::GetMotorCommand() const
{
    return command_;
}


// ================================================================
// Motor feedback
// ================================================================

FishMotorFeedback
BionicFish::ReadMotorFeedback() const
{
    EnsureBound();


    FishMotorFeedback result;


    result.tail.targetPositionRad =
        command_.tailTargetRad;

    result.tail.positionRad =
        tailServo_->getPosition();

    result.tail.velocityRadPerSec =
        tailServo_->getVelocity();

    result.tail.effortNm =
        tailServo_->getEffort();


    result.leftPectoral.targetPositionRad =
        command_
            .leftPectoralTargetRad;

    result.leftPectoral.positionRad =
        leftPectoralServo_
            ->getPosition();

    result.leftPectoral.velocityRadPerSec =
        leftPectoralServo_
            ->getVelocity();

    result.leftPectoral.effortNm =
        leftPectoralServo_
            ->getEffort();


    result.rightPectoral.targetPositionRad =
        command_
            .rightPectoralTargetRad;

    result.rightPectoral.positionRad =
        rightPectoralServo_
            ->getPosition();

    result.rightPectoral.velocityRadPerSec =
        rightPectoralServo_
            ->getVelocity();

    result.rightPectoral.effortNm =
        rightPectoralServo_
            ->getEffort();


    return result;
}


// ================================================================
// Sensor output
// ================================================================

FishSensorData
BionicFish::ReadSensors() const
{
    EnsureBound();


    FishSensorData data;


    // ============================================================
    // IMU
    // ============================================================

    /*
        Stonefish IMU channels：

        0 Roll
        1 Pitch
        2 Yaw

        3 Angular velocity X
        4 Angular velocity Y
        5 Angular velocity Z

        6 Linear acceleration X
        7 Linear acceleration Y
        8 Linear acceleration Z
    */

    data.rollRad =
        ReadChannel(
            imu_,
            0);


    data.pitchRad =
        ReadChannel(
            imu_,
            1);


    data.yawRad =
        ReadChannel(
            imu_,
            2);


    data.angularVelocityRadPerSec =
        sf::Vector3(

            ReadChannel(
                imu_,
                3),

            ReadChannel(
                imu_,
                4),

            ReadChannel(
                imu_,
                5));


    data.linearAccelerationMps2 =
        sf::Vector3(

            ReadChannel(
                imu_,
                6),

            ReadChannel(
                imu_,
                7),

            ReadChannel(
                imu_,
                8));


    // ============================================================
    // Pressure / depth
    // ============================================================

    data.pressurePa =
        ReadChannel(
            pressure_,
            0);


    const sf::Scalar denominator =
        waterDensity_
        * gravityMagnitude_;


    if (denominator > 0.0)
    {
        data.depthMeters =
            data.pressurePa
            / denominator;
    }


    // ============================================================
    // Encoders
    // ============================================================

    data.tailEncoder.angleRad =
        ReadChannel(
            tailEncoder_,
            0);


    data.tailEncoder
        .angularVelocityRadPerSec =
            ReadChannel(
                tailEncoder_,
                1);


    data.leftPectoralEncoder.angleRad =
        ReadChannel(
            leftPectoralEncoder_,
            0);


    data.leftPectoralEncoder
        .angularVelocityRadPerSec =
            ReadChannel(
                leftPectoralEncoder_,
                1);


    data.rightPectoralEncoder.angleRad =
        ReadChannel(
            rightPectoralEncoder_,
            0);


    data.rightPectoralEncoder
        .angularVelocityRadPerSec =
            ReadChannel(
                rightPectoralEncoder_,
                1);


    // ============================================================
    // Sonar
    // ============================================================

    const unsigned int beamCount =
        sonar_->getNumOfChannels();


    data.sonarRangesMeters.reserve(
        beamCount);


    for (
        unsigned int beam = 0;
        beam < beamCount;
        ++beam)
    {
        data.sonarRangesMeters.push_back(
            sonar_->getLastValue(
                beam));
    }


    return data;
}


sf::SolidEntity*
BionicFish::GetBody() const
{
    return body_;
}
