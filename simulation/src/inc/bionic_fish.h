#pragma once

#include <Stonefish/StonefishCommon.h>

#include <string>
#include <vector>


namespace sf
{
class SimulationManager;
class Robot;
class SolidEntity;
class Servo;
class ScalarSensor;
}


/*
    ================================================================
    Motor command
    ================================================================

    第一版所有命令都使用 SI：

        rad
        rad/s
        Nm

    当前三个电机均使用 Servo POSITION mode。

    所以输入是：

        target joint angle [rad]

    后面如果真实硬件接受：
        PWM
        current
        CAN command
        Dynamixel position

    只需要增加 hardware adapter。
*/
struct FishMotorCommand
{
    sf::Scalar tailTargetRad = 0.0;

    sf::Scalar leftPectoralTargetRad = 0.0;

    sf::Scalar rightPectoralTargetRad = 0.0;
};


/*
    单个电机的真实反馈。

    targetPositionRad:
        我们发给虚拟电机的目标

    positionRad:
        电机/关节实际位置

    velocityRadPerSec:
        实际角速度

    effortNm:
        Servo 当前实际输出力矩
*/
struct FishMotorChannelFeedback
{
    sf::Scalar targetPositionRad = 0.0;

    sf::Scalar positionRad = 0.0;

    sf::Scalar velocityRadPerSec = 0.0;

    sf::Scalar effortNm = 0.0;
};


struct FishMotorFeedback
{
    FishMotorChannelFeedback tail;

    FishMotorChannelFeedback leftPectoral;

    FishMotorChannelFeedback rightPectoral;
};


/*
    编码器输出。
*/
struct FishEncoderData
{
    sf::Scalar angleRad = 0.0;

    sf::Scalar angularVelocityRadPerSec = 0.0;
};


/*
    ================================================================
    Virtual hardware sensor frame
    ================================================================

    这里故意不放：

        world position
        obstacle world position
        perfect world velocity
        river chunk information

    因为这些东西真实机器人无法直接读取。

    RL 以后应该主要使用这个结构。
*/
struct FishSensorData
{
    // ------------------------------------------------------------
    // IMU
    // ------------------------------------------------------------

    sf::Scalar rollRad = 0.0;

    sf::Scalar pitchRad = 0.0;

    sf::Scalar yawRad = 0.0;


    sf::Vector3 angularVelocityRadPerSec =
        sf::Vector3(0.0, 0.0, 0.0);


    sf::Vector3 linearAccelerationMps2 =
        sf::Vector3(0.0, 0.0, 0.0);


    // ------------------------------------------------------------
    // Pressure / depth
    // ------------------------------------------------------------

    /*
        Stonefish Pressure sensor 返回 gauge pressure [Pa].
    */
    sf::Scalar pressurePa = 0.0;


    /*
        根据：

            depth = pressure / (rho * g)

        得到的便利值。

        原始硬件值仍然保留在 pressurePa。
    */
    sf::Scalar depthMeters = 0.0;


    // ------------------------------------------------------------
    // Encoders
    // ------------------------------------------------------------

    FishEncoderData tailEncoder;

    FishEncoderData leftPectoralEncoder;

    FishEncoderData rightPectoralEncoder;


    // ------------------------------------------------------------
    // Sonar
    // ------------------------------------------------------------

    /*
        1D multibeam range values [m].

        当前配置：

            FOV = 90°
            steps = 18

        因此通常有 19 个距离值。
    */
    std::vector<sf::Scalar>
        sonarRangesMeters;
};


class BionicFish final
{
public:

    BionicFish() = default;


    /*
        ScenarioParser 创建 BionicFish 后，
        通过名称绑定所有虚拟硬件。
    */
    void Bind(
        sf::SimulationManager& simulation,
        const std::string& robotName = "BionicFish");


    bool IsBound() const;


    /*
        ============================================================
        Virtual motor input
        ============================================================
    */

    void SetMotorCommand(
        const FishMotorCommand& command);


    void SetTailTarget(
        sf::Scalar targetRad);


    void SetPectoralTargets(
        sf::Scalar leftRad,
        sf::Scalar rightRad);


    FishMotorCommand
    GetMotorCommand() const;


    /*
        ============================================================
        Virtual hardware output
        ============================================================
    */

    FishMotorFeedback
    ReadMotorFeedback() const;


    FishSensorData
    ReadSensors() const;


    /*
        Base body。

        这里只给 simulator/debug/camera 使用。
        后面不要直接把它的 world pose 喂给 RL。
    */
    sf::SolidEntity*
    GetBody() const;


private:

    void EnsureBound() const;


    // ------------------------------------------------------------
    // Robot
    // ------------------------------------------------------------

    std::string robotName_;

    sf::Robot* robot_ = nullptr;

    sf::SolidEntity* body_ = nullptr;


    // ------------------------------------------------------------
    // Motors
    // ------------------------------------------------------------

    sf::Servo* tailServo_ = nullptr;

    sf::Servo* leftPectoralServo_ = nullptr;

    sf::Servo* rightPectoralServo_ = nullptr;


    FishMotorCommand command_;


    // ------------------------------------------------------------
    // Sensors
    // ------------------------------------------------------------

    sf::ScalarSensor* imu_ = nullptr;

    sf::ScalarSensor* pressure_ = nullptr;

    sf::ScalarSensor* sonar_ = nullptr;

    sf::ScalarSensor* tailEncoder_ = nullptr;

    sf::ScalarSensor* leftPectoralEncoder_ = nullptr;

    sf::ScalarSensor* rightPectoralEncoder_ = nullptr;


    // ------------------------------------------------------------
    // Environment data used only for pressure -> depth conversion
    // ------------------------------------------------------------

    sf::Scalar waterDensity_ = 997.0;

    sf::Scalar gravityMagnitude_ = 9.81;
};
