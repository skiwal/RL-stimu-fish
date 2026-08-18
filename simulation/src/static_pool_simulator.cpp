#include "inc/static_pool_simulator.h"

#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <iostream>
#include <stdexcept>
#include <string>


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
        ============================================================
        Stage 1
        ============================================================

        We intentionally KEEP PoolTestMode in the interface so that
        pool_main.cpp does not need to change.

        However Stage 1 does NOT use the mode to control anything.

        Regardless of whether the command line says:

            neutral
            straight
            dive
            roll_left
            ...

        Stage 1 always behaves as a FIXED geometry test.

        Disabled:

            TailMotor               OFF
            LeftPectoralMotor       OFF
            RightPectoralMotor      OFF

            passive tail springs    OFF

            scripted swimming       OFF

            ApplyForce()            NEVER USED
    */

    setCallSimulationStepCompleted(
        false);
}


void StaticPoolSimulator::BuildScenario()
{
    // ============================================================
    // Stonefish application
    // ============================================================

    sf::SimulationApp* app =
        sf::SimulationApp::getApp();


    if (app == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: SimulationApp is not available.");
    }


    // ============================================================
    // Use the ORIGINAL static pool.
    //
    // This pool has already been visually validated.
    //
    // The only important change inside static_pool.scn is that the
    // old primitive fish include has been replaced with:
    //
    //     robots/bionic_fish/bionic_fish.scn
    //
    // and robot_fixed=true.
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
    // Parse scenario
    // ============================================================

    sf::ScenarioParser parser(
        this);


    if (!parser.Parse(
            scenarioPath))
    {
        throw std::runtime_error(
            "StaticPoolSimulator: failed to parse scenario: "
            + scenarioPath);
    }


    // ============================================================
    // Bind real BionicFish
    // ============================================================

    BindBionicFish();


    // ============================================================
    // Camera follows real fish Body.
    //
    // Because the whole robot is fixed during Stage 1, an unstable
    // dynamic state cannot drag the camera out of the pool.
    // ============================================================

    sf::OpenGLTrackball* trackball =
        getTrackball();


    if (
        trackball != nullptr
        && fishBody_ != nullptr)
    {
        trackball->GlueToMoving(
            fishBody_);
    }


    // ============================================================
    // Stage 1 information
    // ============================================================

    std::cout
        << "\n"
        << "============================================================\n"
        << " BionicFish V1 - Stage 1 Fixed Geometry Test\n"
        << "============================================================\n"
        << "\n"
        << "Scenario:\n"
        << "  env/static_pool.scn\n"
        << "\n"
        << "Robot:\n"
        << "  BionicFish\n"
        << "\n"
        << "Command-line test mode:\n"
        << "  "
        << PoolTestModeName(
               mode_)
        << "\n"
        << "\n"
        << "IMPORTANT:\n"
        << "  Stage 1 intentionally ignores the test mode.\n"
        << "\n"
        << "Stage 1 configuration:\n"
        << "  robot fixed             YES\n"
        << "  motor controller        OFF\n"
        << "  passive tail spring     OFF\n"
        << "  scripted swimming       OFF\n"
        << "  RL                      OFF\n"
        << "  fake body force         OFF\n"
        << "\n"
        << "Robot binding:\n"
        << "  BionicFish              OK\n"
        << "  Body                    OK\n"
        << "\n"
        << "Expected result:\n"
        << "  fish remains at initial position\n"
        << "  fish does not translate\n"
        << "  fish does not rotate\n"
        << "  tail does not move\n"
        << "  pectoral fins do not move\n"
        << "============================================================\n"
        << std::endl;
}


void StaticPoolSimulator::BindBionicFish()
{
    // ============================================================
    // Find BionicFish
    // ============================================================

    fishRobot_ =
        getRobot(
            "BionicFish");


    if (fishRobot_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: robot 'BionicFish' was not found.\n"
            "Check the include section at the end of static_pool.scn.");
    }


    // ============================================================
    // Base link = Body
    // ============================================================

    fishBody_ =
        fishRobot_
            ->getBaseLink();


    if (fishBody_ == nullptr)
    {
        throw std::runtime_error(
            "StaticPoolSimulator: BionicFish base link was not found.");
    }


    std::cout
        << "StaticPoolSimulator: "
        << "BionicFish successfully bound."
        << std::endl;
}


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
