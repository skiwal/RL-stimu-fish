#pragma once

#include <Stonefish/core/SimulationManager.h>


namespace sf
{
class Robot;
class SolidEntity;
}


/*
    ================================================================
    Static-pool test modes
    ================================================================

    Stage 1 currently ignores these modes because:

        robot is fixed
        motors are disabled
        passive tail springs are disabled

    We KEEP this interface because later stages will use it again.
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
    Stage 1 — Fixed BionicFish geometry test
    ================================================================

    Goal:

        1. Load the original static_pool.scn.
        2. Load the real mesh-based BionicFish.
        3. Keep the whole robot fixed.
        4. Disable all motor/spring control.
        5. Verify:
             - pool rendering
             - fish rendering
             - link assembly
             - camera tracking

    No propulsion.
    No passive-tail spring.
    No RL.
    No ApplyForce().
*/
class StaticPoolSimulator final
    : public sf::SimulationManager
{
public:

    StaticPoolSimulator(
        sf::Scalar stepsPerSecond,
        PoolTestMode mode);


    void BuildScenario() override;


    sf::Robot*
    GetFishRobot() const;


    sf::SolidEntity*
    GetFishBody() const;


    PoolTestMode
    GetTestMode() const;


private:

    void BindBionicFish();


    PoolTestMode mode_ =
        PoolTestMode::Neutral;


    sf::Robot* fishRobot_ =
        nullptr;


    sf::SolidEntity* fishBody_ =
        nullptr;
};
