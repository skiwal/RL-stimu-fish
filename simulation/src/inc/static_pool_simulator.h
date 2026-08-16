#pragma once

#include <Stonefish/core/SimulationManager.h>

#include "bionic_fish.h"


/*
    ================================================================
    Built-in test modes
    ================================================================

    这些不是最终控制器。

    目的只是验证：

        tail hydrodynamics
        steering
        pitch
        roll
        virtual sensors
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

    /*
        External 模式不会自动生成控制命令。

        后面：

            RL
            Python bridge
            ROS2
            hardware interface

        都可以通过 SetExternalMotorCommand()
        写入三个 motor target。
    */
    External
};


const char*
PoolTestModeName(
    PoolTestMode mode);


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


    /*
        ============================================================
        External control interface
        ============================================================
    */

    void SetExternalMotorCommand(
        const FishMotorCommand& command);


    FishSensorData
    ReadFishSensors() const;


    FishMotorFeedback
    ReadFishMotorFeedback() const;


    BionicFish&
    GetFish();


    const BionicFish&
    GetFish() const;


private:

    FishMotorCommand
    MakeScriptedCommand(
        sf::Scalar testTime) const;


    void PrintTelemetry();


    PoolTestMode mode_;


    BionicFish fish_;


    FishMotorCommand
        externalCommand_;


    sf::Scalar elapsedTime_ =
        0.0;


    sf::Scalar lastTelemetryTime_ =
        -1.0;
};
