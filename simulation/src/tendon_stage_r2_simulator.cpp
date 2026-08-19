#include "inc/tendon_stage_r2_simulator.h"
#include "inc/tendon_tail_actuator.h"

#include <Stonefish/core/FeatherstoneRobot.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/core/SimulationApp.h>

#include <Stonefish/entities/FeatherstoneEntity.h>
#include <Stonefish/entities/SolidEntity.h>

#include <Stonefish/graphics/OpenGLTrackball.h>

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>


namespace
{

constexpr sf::Scalar kRadToDeg =
    57.2957795130823208768;

}


TendonStageR2Simulator::TendonStageR2Simulator(
    sf::Scalar stepsPerSecond)
    : sf::SimulationManager(
          stepsPerSecond)
{
    setCallSimulationStepCompleted(
        true);
}


void
TendonStageR2Simulator::BuildScenario()
{
    sf::SimulationApp*
        app =
            sf::SimulationApp::getApp();


    if (
        app == nullptr)
    {
        throw std::runtime_error(
            "TendonStageR2Simulator: SimulationApp unavailable.");
    }


    const std::string
        scenario =
            app->getDataPath()
            +
            "env/static_pool.scn";


    std::cout
        << "\nLoading:\n  "
        << scenario
        << "\n";


    sf::ScenarioParser
        parser(
            this);


    if (
        !parser.Parse(
            scenario))
    {
        throw std::runtime_error(
            "Failed to parse static_pool.scn");
    }


    BindFish();


    SetNeutralInitialCondition();


    RegisterTendonActuator();


    OpenCsv();


    ConfigureCamera();


    const auto
        snapshot =
            tendonActuator_->GetSnapshot();


    std::cout
        << "\n"
        << "============================================================\n"
        << " BionicFish Direct Tendon Force Test\n"
        << "============================================================\n"
        << "Body fixed                 YES\n"
        << "M1 motor                    DISABLED\n"
        << "Motor shaft                 NOT USED\n"
        << "Crank dynamics              NOT USED\n"
        << "Tendon elasticity           NOT USED\n"
        << "Tendon damping              NOT USED\n"
        << "Length -> tension            NO\n"
        << "Direct tendon tension        YES\n"
        << "Test tension                 1.0 N\n"
        << "Routing                      0,0,0,1,1\n"
        << "Passive joint stiffness      0.65 Nm/rad\n"
        << "Passive joint damping        0\n"
        << "Joint safety                 60 deg\n"
        << "Left initial path length     "
        << snapshot.initialTendonLengthM[0]
        << " m\n"
        << "Right initial path length    "
        << snapshot.initialTendonLengthM[1]
        << " m\n"
        << "CSV                          direct_tendon_force_test.csv\n"
        << "============================================================\n"
        << std::endl;
}


void
TendonStageR2Simulator::BindFish()
{
    fishRobot_ =
        getRobot(
            "BionicFish");


    if (
        fishRobot_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish not found.");
    }


    fishFeatherstoneRobot_ =
        dynamic_cast<sf::FeatherstoneRobot*>(
            fishRobot_);


    if (
        fishFeatherstoneRobot_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish is not a FeatherstoneRobot.");
    }


    fishDynamics_ =
        fishFeatherstoneRobot_->getDynamics();


    fishBody_ =
        fishRobot_->getBaseLink();


    if (
        fishDynamics_ == nullptr
        ||
        fishBody_ == nullptr)
    {
        throw std::runtime_error(
            "BionicFish dynamics/base unavailable.");
    }


    // ============================================================
    // Body link
    // ============================================================

    bodyLinkIndex_ =
        FindLinkIndex(
            "Body");


    if (
        bodyLinkIndex_ < 0)
    {
        throw std::runtime_error(
            "Cannot find Body link.");
    }


    // ============================================================
    // Tail joints and links
    // ============================================================

    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        const std::string
            jointName =
                "TailJoint"
                +
                std::to_string(
                    i);


        const std::string
            linkName =
                "Tail"
                +
                std::to_string(
                    i);


        tailJointIndices_[i] =
            FindJointIndex(
                jointName.c_str());


        tailLinkIndices_[i] =
            FindLinkIndex(
                linkName.c_str());


        if (
            tailJointIndices_[i] < 0)
        {
            throw std::runtime_error(
                "Cannot find "
                +
                jointName);
        }


        if (
            tailLinkIndices_[i] < 0)
        {
            throw std::runtime_error(
                "Cannot find "
                +
                linkName);
        }
    }


    std::cout
        << "\nDirect tendon test binding:\n"
        << "  Body -> link index "
        << bodyLinkIndex_
        << "\n";


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        std::cout
            << "  TailJoint"
            << i
            << " -> joint index "
            << tailJointIndices_[i]
            << " | Tail"
            << i
            << " -> link index "
            << tailLinkIndices_[i]
            << "\n";
    }


    std::cout
        << std::endl;
}


int
TendonStageR2Simulator::FindJointIndex(
    const char* jointName) const
{
    if (
        fishDynamics_ == nullptr
        ||
        jointName == nullptr)
    {
        return -1;
    }


    const std::string
        shortName(
            jointName);


    const std::string
        fullName =
            "BionicFish/"
            +
            shortName;


    const unsigned int
        count =
            fishDynamics_->getNumOfJoints();


    for (
        unsigned int i = 0;
        i < count;
        ++i)
    {
        const std::string
            actual =
                fishDynamics_->getJointName(
                    i);


        if (
            actual == shortName
            ||
            actual == fullName)
        {
            return
                static_cast<int>(
                    i);
        }
    }


    return -1;
}


int
TendonStageR2Simulator::FindLinkIndex(
    const char* linkName) const
{
    if (
        fishDynamics_ == nullptr
        ||
        linkName == nullptr)
    {
        return -1;
    }


    const std::string
        shortName(
            linkName);


    const std::string
        fullName =
            "BionicFish/"
            +
            shortName;


    const unsigned int
        count =
            fishDynamics_->getNumOfLinks();


    // IMPORTANT:
    //
    // Search the FeatherstoneEntity link array directly.
    //
    // These are the indices expected by:
    //
    //     AddLinkForce()
    //     AddLinkTorque()
    //     getLinkTransform()

    for (
        unsigned int i = 0;
        i < count;
        ++i)
    {
        const sf::FeatherstoneLink
            link =
                fishDynamics_->getLink(
                    i);


        if (
            link.solid == nullptr)
        {
            continue;
        }


        const std::string
            actual =
                link.solid->getName();


        if (
            actual == shortName
            ||
            actual == fullName)
        {
            return
                static_cast<int>(
                    i);
        }
    }


    return -1;
}


void
TendonStageR2Simulator::SetNeutralInitialCondition()
{
    for (
        int index
        : tailJointIndices_)
    {
        fishDynamics_->setJointIC(
            static_cast<unsigned int>(
                index),
            0.0,
            0.0);
    }
}


void
TendonStageR2Simulator::RegisterTendonActuator()
{
    TendonTailParameters
        parameters;


    // ============================================================
    // Passive tail
    // ============================================================

    parameters.passiveStiffnessNmRad =
        0.65;


    parameters.passiveDampingNmsRad =
        0.0;


    // ============================================================
    // Direct tendon experiment
    // ============================================================

    parameters.directTestTensionN =
        1.0;


    parameters.initialSettleTimeS =
        1.0;


    parameters.rampTimeS =
        1.0;


    parameters.holdTimeS =
        1.0;


    parameters.centerPauseTimeS =
        1.0;


    parameters.jointSafetyLimitRad =
        1.0471975511965976;


    std::array<unsigned int, 5>
        jointIndices {};


    std::array<unsigned int, 5>
        linkIndices {};


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        jointIndices[i] =
            static_cast<unsigned int>(
                tailJointIndices_[i]);


        linkIndices[i] =
            static_cast<unsigned int>(
                tailLinkIndices_[i]);
    }


    tendonActuator_ =
        new TendonTailActuator(
            "BionicFish/DirectTendonForceTest",
            fishDynamics_,
            static_cast<unsigned int>(
                bodyLinkIndex_),
            linkIndices,
            jointIndices,
            parameters);


    AddActuator(
        tendonActuator_);
}


void
TendonStageR2Simulator::ConfigureCamera()
{
    sf::OpenGLTrackball*
        trackball =
            getTrackball();


    if (
        trackball == nullptr
        ||
        fishBody_ == nullptr)
    {
        return;
    }


    trackball->GlueToMoving(
        fishBody_);


    trackball->UpdateCenterPos();


    trackball->MouseScroll(
        -10.5f);


    trackball->UpdateTransform();
}


void
TendonStageR2Simulator::OpenCsv()
{
    csv_.open(
        "direct_tendon_force_test.csv",
        std::ios::out
        |
        std::ios::trunc);


    if (
        !csv_.is_open())
    {
        throw std::runtime_error(
            "Cannot create direct_tendon_force_test.csv");
    }


    csv_
        << "time_s,"
        << "phase,"
        << "left_tension_cmd_n,"
        << "right_tension_cmd_n,"
        << "left_length_m,"
        << "right_length_m,"
        << "left_initial_length_m,"
        << "right_initial_length_m,"
        << "left_length_change_m,"
        << "right_length_change_m,";


    for (
        int i = 0;
        i < 5;
        ++i)
    {
        csv_
            << "j"
            << i
            << "_q_rad,"

            << "j"
            << i
            << "_q_deg,"

            << "j"
            << i
            << "_qdot_rad_s,"

            << "j"
            << i
            << "_passive_tau_nm,";
    }


    csv_
        << "safety_tripped\n";


    csv_
        << std::setprecision(
            10);
}


void
TendonStageR2Simulator::RecordSample()
{
    if (
        !csv_.is_open()
        ||
        tendonActuator_ == nullptr)
    {
        return;
    }


    const auto
        s =
            tendonActuator_->GetSnapshot();


    csv_
        << s.timeS
        << ","

        << s.testPhase
        << ","

        << s.commandedTensionN[0]
        << ","

        << s.commandedTensionN[1]
        << ","

        << s.tendonLengthM[0]
        << ","

        << s.tendonLengthM[1]
        << ","

        << s.initialTendonLengthM[0]
        << ","

        << s.initialTendonLengthM[1]
        << ","

        << s.tendonLengthChangeM[0]
        << ","

        << s.tendonLengthChangeM[1]
        << ",";


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        csv_
            << s.jointPositionRad[i]
            << ","

            << s.jointPositionRad[i]
                *
                kRadToDeg
            << ","

            << s.jointVelocityRadS[i]
            << ","

            << s.passiveTorqueNm[i]
            << ",";
    }


    csv_
        << (
            s.safetyTripped
                ?
            1
                :
            0
        )
        << "\n";
}


void
TendonStageR2Simulator::PrintTelemetry()
{
    if (
        tendonActuator_ == nullptr)
    {
        return;
    }


    const auto
        s =
            tendonActuator_->GetSnapshot();


    std::cout
        << std::fixed
        << std::setprecision(
            3)

        << "[DIRECT] t="
        << s.timeS

        << " phase="
        << s.testPhase

        << " | T=("
        << s.commandedTensionN[0]
        << ","
        << s.commandedTensionN[1]
        << ")N"

        << " | dL=("
        << s.tendonLengthChangeM[0]
            *
            1000.0
        << ","
        << s.tendonLengthChangeM[1]
            *
            1000.0
        << ")mm"

        << " | J=(";


    for (
        std::size_t i = 0;
        i < 5;
        ++i)
    {
        if (
            i != 0)
        {
            std::cout
                << ",";
        }


        std::cout
            << s.jointPositionRad[i]
                *
                kRadToDeg;
    }


    std::cout
        << ")deg"

        << " | safety="
        << (
            s.safetyTripped
                ?
            "TRIPPED"
                :
            "OK"
        )

        << std::endl;
}


void
TendonStageR2Simulator::SimulationStepCompleted(
    sf::Scalar timeStep)
{
    elapsedTimeS_ +=
        timeStep;


    if (
        lastCsvTimeS_ < 0.0
        ||
        elapsedTimeS_
            -
        lastCsvTimeS_
            >=
        csvPeriodS_)
    {
        RecordSample();


        lastCsvTimeS_ =
            elapsedTimeS_;
    }


    if (
        lastConsoleTimeS_ < 0.0
        ||
        elapsedTimeS_
            -
        lastConsoleTimeS_
            >=
        consolePeriodS_)
    {
        PrintTelemetry();


        lastConsoleTimeS_ =
            elapsedTimeS_;
    }
}
